// AI characterization harness for the OpenTrickler simulator.
//
// Runs a full AI powder characterization through the firmware's real
// charge_mode state machine (the AI sample path in charge_mode.cpp plus
// ai_tuning.c), exactly as the web GUI's "Start characterization" does, then
// applies the fitted model and throws normal charges with it.
//
// A watchdog fails the run if any single charge-mode phase sits for longer
// than --stall-s (e.g. charge mode falling through to its normal "Remove Cup"
// wait because a characterization drop was rejected).
//
// Exit status: 0 = characterization completed and model applied,
//              2 = stalled, 3 = AI tuning reported an error / never completed.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>

#include <FreeRTOS.h>
#include <task.h>

#include "charge_mode.h"
#include "profile.h"
#include "ai_tuning.h"
#include "motors.h"
#include "eeprom.h"
#include "flash_storage.h"

#include "sim_plant.h"
#include "sim_harness.h"

extern "C" {
#include "scale.h"
void sim_rt_start(void);
void sim_rt_stop(void);
}

extern sim_plant_config_t g_plant_cfg;
extern sim_plant_state_t g_plant_state;
extern scale_config_t scale_config;
extern charge_mode_config_t charge_mode_config;
std::recursive_mutex &sim_plant_mutex();

extern void charge_mode_wait_for_complete(void);
extern void charge_mode_stabilize(void);
extern void charge_mode_wait_for_cup_removal(void);

extern "C" { int g_sim_verbosity = 1; }
static float g_last_coarse_rps = 0.0f;
static TickType_t g_coarse_stop_tick = 0;
extern "C" void sim_harness_on_motor_command(motor_select_t motor, float rps) {
    if (motor == SELECT_COARSE_TRICKLER_MOTOR) {
        if (rps <= 0.0f && g_last_coarse_rps > 0.0f) {
            g_coarse_stop_tick = xTaskGetTickCount();
        }
        g_last_coarse_rps = rps;
    }
    if (g_sim_verbosity >= 2) {
        printf("      [%7u ms] %s -> %.3f rps  (pan %.3f gn)\n",
               (unsigned)xTaskGetTickCount(),
               motor == SELECT_COARSE_TRICKLER_MOTOR ? "coarse" : "fine  ",
               rps, g_plant_state.true_weight_gn);
    }
}

// The firmware tares between characterization samples through
// scale_handle->force_zero(); route it to the simulated scale.
static void sim_force_zero(void) {
    std::lock_guard<std::recursive_mutex> lock(sim_plant_mutex());
    sim_plant_zero(&g_plant_state);
}
static scale_handle_t g_sim_scale_handle;

static std::atomic<uint32_t> g_phase_start_tick{0};
static std::atomic<int> g_phase_state{-1};
static float g_stall_s = 60.0f;

static const char *state_name(int s) {
    switch (s) {
        case CHARGE_MODE_WAIT_FOR_ZERO: return "WAIT_FOR_ZERO";
        case CHARGE_MODE_WAIT_FOR_COMPLETE: return "WAIT_FOR_COMPLETE";
        case CHARGE_MODE_STABILIZING: return "STABILIZING";
        case CHARGE_MODE_WAIT_FOR_CUP_REMOVAL: return "WAIT_FOR_CUP_REMOVAL";
        case CHARGE_MODE_WAIT_FOR_CUP_RETURN: return "WAIT_FOR_CUP_RETURN";
        case CHARGE_MODE_EXIT: return "EXIT";
        default: return "?";
    }
}

static void print_session(const char *prefix) {
    static ai_tuning_session_t s;
    if (!ai_tuning_get_session_copy(&s)) {
        printf("%s(session unavailable)\n", prefix);
        return;
    }
    printf("%sai state %d  drops %u/%u  status \"%s\"%s%s%s\n", prefix,
           (int)s.state, s.drops_completed, s.total_samples_planned,
           s.status_message,
           s.error_message[0] ? "  error \"" : "", s.error_message,
           s.error_message[0] ? "\"" : "");
}

static void watchdog_thread() {
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        uint32_t now = xTaskGetTickCount();
        int st = g_phase_state.load();
        if (st < 0) continue;
        if ((float)(now - g_phase_start_tick.load()) / 1000.0f > g_stall_s) {
            printf("\n!!! STALL: charge mode stuck in %s for over %.0f s\n",
                   state_name(st), g_stall_s);
            print_session("    ");
            fflush(stdout);
            std::_Exit(2);
        }
    }
}

static void setup_profile(void) {
    profile_data_init();
    profile_t *p = profile_get_selected();
    if (p == NULL) return;
    snprintf(p->name, sizeof(p->name), "SimProfile");
    p->coarse_kp = 0.35f;  p->coarse_ki = 0.0f; p->coarse_kd = 0.0f;
    p->fine_kp = 3.0f;     p->fine_ki = 0.0f;   p->fine_kd = 0.0f;
    p->coarse_min_flow_speed_rps = 0.30f;
    p->coarse_max_flow_speed_rps = 8.0f;
    p->fine_min_flow_speed_rps = 0.08f;
    p->fine_max_flow_speed_rps = 2.20f;
}

// Run charge mode's state machine (as charge_mode_menu() does) until the AI
// session is no longer active, printing each recorded drop.
static bool run_characterization(void) {
    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_COMPLETE;
    uint8_t last_drops = 0;
    while (true) {
        int st = (int)charge_mode_config.charge_mode_state;
        g_phase_state.store(st);
        g_phase_start_tick.store(xTaskGetTickCount());
        switch (st) {
            case CHARGE_MODE_WAIT_FOR_COMPLETE: charge_mode_wait_for_complete(); break;
            case CHARGE_MODE_STABILIZING:       charge_mode_stabilize(); break;
            case CHARGE_MODE_WAIT_FOR_CUP_REMOVAL: {
                charge_mode_wait_for_cup_removal();
                static ai_tuning_session_t s;
                if (ai_tuning_get_session_copy(&s)) {
                    if (s.drops_completed != last_drops && g_sim_verbosity >= 1) {
                        const ai_drop_telemetry_t &d =
                            s.drops[(s.drop_write_idx + AI_TUNING_DROP_BUF_SIZE - 1) % AI_TUNING_DROP_BUF_SIZE];
                        printf("  drop %2u/%-2u %s %5.2f rps %6.0f ms  delivered %6.3f  tail %6.3f gn%s\n",
                               s.drops_completed, s.total_samples_planned,
                               d.motor_mode == AI_MOTOR_MODE_FINE_ONLY ? "fine  " : "coarse",
                               d.speed_rps, d.motor_on_time_ms, d.delivered_weight,
                               d.tail_weight, d.settled ? "" : "  (not settled)");
                    }
                    last_drops = s.drops_completed;
                    if (s.state == AI_TUNING_ERROR) {
                        g_phase_state.store(-1);
                        print_session("  ");
                        return false;
                    }
                }
                break;
            }
            case CHARGE_MODE_EXIT:
            default:
                g_phase_state.store(-1);
                return ai_tuning_is_complete();
        }
        if (!ai_tuning_is_active() && ai_tuning_is_complete()) {
            g_phase_state.store(-1);
            return true;
        }
    }
}

static void print_model(void) {
    static ai_profile_model_t m;
    if (!ai_tuning_get_profile_model_copy((uint8_t)profile_get_selected_idx(), &m)) {
        printf("  (no saved model)\n");
        return;
    }
    printf("\n=== Fitted model (valid %d, enabled %d) ===\n", m.valid, m.enabled);
    printf("  coarse: %u samples  slope %.3f  intercept %.3f  tail %.3f gn  best %.2f rps @ %.3f gn/s\n",
           m.coarse_sample_count, m.coarse_flow_slope, m.coarse_flow_intercept,
           m.coarse_tail_gn, m.coarse_best_speed_rps, m.coarse_best_flow_gps);
    printf("          trim %.2f rps @ %.3f gn/s, tail %.3f gn\n",
           m.coarse_trim_speed_rps, m.coarse_trim_flow_gps, m.coarse_trim_tail_gn);
    printf("  fine:   %u samples  slope %.3f  intercept %.3f  tail %.3f gn  best %.2f rps @ %.3f gn/s\n",
           m.fine_sample_count, m.fine_flow_slope, m.fine_flow_intercept,
           m.fine_tail_gn, m.fine_best_speed_rps, m.fine_best_flow_gps);
    printf("          recovery %.2f rps @ %.4f gn/s (%u samples), window %.3f gn\n",
           m.fine_recovery_speed_rps, m.fine_recovery_flow_gps,
           m.fine_recovery_sample_count, m.recommended_fine_window_gn);
    printf("  kernel estimate %.4f gn (confidence %.2f)  fine tube profile %s\n",
           m.estimated_kernel_weight_gn, m.kernel_weight_confidence,
           ai_tuning_fine_tube_profile_to_string((ai_fine_tube_profile_t)m.fine_tube_profile));
    if (g_sim_verbosity >= 1) {
        printf("  stored coarse samples (speed, on-time, delivered, tail, flow):\n");
        for (int i = 0; i < m.coarse_sample_count; i++) {
            const ai_flow_sample_t &c = m.coarse_samples[i];
            double true_flow = g_plant_cfg.coarse_flow_k * pow(c.speed_rps, g_plant_cfg.coarse_flow_exponent);
            printf("    %5.2f rps %5.0f ms  %6.3f gn  tail %6.3f  flow %6.3f gn/s (plant %5.3f)\n",
                   c.speed_rps, c.motor_on_time_ms, c.delivered_weight, c.tail_weight,
                   c.flow_gps, true_flow);
        }
    }
    printf("  plant truth: coarse tail %.3f gn, fine tail %.3f gn, kernel %.4f gn\n",
           g_plant_cfg.coarse_tail_gn, g_plant_cfg.fine_tail_gn, g_plant_cfg.kernel_weight_gn);
}

static void run_charges(float target, int n) {
    if (n <= 0) return;
    printf("\n=== %d charges at %.2f gn, %s controller ===\n", n, target,
           charge_mode_config.eeprom_charge_mode_data.use_adaptive_controller
               ? "adaptive (AI model)" : "PID (profile values)");
    double sum = 0, sum_sq = 0, tsum = 0, worst = 0;
    int within = 0;
    float tol = charge_mode_config.eeprom_charge_mode_data.accept_tolerance_gn;
    if (tol <= 0.0f) tol = 0.02f;
    for (int i = 0; i < n; i++) {
        {
            std::lock_guard<std::recursive_mutex> lock(sim_plant_mutex());
            sim_plant_empty_pan(&g_plant_state);
            // Re-zero like the cup return / wait-for-zero step does on the
            // device, or the scale's zero drift builds up from charge to
            // charge and every charge lands short by that much.
            sim_plant_zero(&g_plant_state);
        }
        vTaskDelay(500);
        g_coarse_stop_tick = 0;
        charge_mode_config.target_charge_weight = target;
        charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_COMPLETE;
        g_phase_state.store(CHARGE_MODE_WAIT_FOR_COMPLETE);
        g_phase_start_tick.store(xTaskGetTickCount());
        TickType_t start = xTaskGetTickCount();
        charge_mode_wait_for_complete();
        charge_mode_stabilize();
        TickType_t end = xTaskGetTickCount();
        g_phase_state.store(-1);
        vTaskDelay(1500);
        double err = g_plant_state.true_weight_gn - target;
        double secs = (end - start) / 1000.0;
        double coarse_s = (g_coarse_stop_tick > start) ? (g_coarse_stop_tick - start) / 1000.0 : 0.0;
        sum += err; sum_sq += err * err; tsum += secs;
        if (fabs(err) > fabs(worst)) worst = err;
        if (fabs(err) <= tol) within++;
        if (g_sim_verbosity >= 1) {
            printf("  charge %2d: err %+7.3f gn  %5.2f s  (coarse %5.2f s, fine %5.2f s)\n",
                   i + 1, err, secs, coarse_s, secs - coarse_s);
        }
    }
    double mean = sum / n;
    double sd = n > 1 ? sqrt(fmax(0.0, (sum_sq - n * mean * mean) / (n - 1))) : 0.0;
    printf("  mean err %+.3f gn  sd %.3f  worst %+.3f  within +/-%.3f: %d/%d  mean time %.2f s\n",
           mean, sd, worst, tol, within, n, tsum / n);
}

// The simulated flash is RAM, so a fitted model only lives for one process.
// These save/restore the raw AI history so charges can be re-run against a
// model without repeating the ~3 minute characterization.
static bool save_model_file(const char *path) {
    static ai_tuning_history_t h;
    if (!flash_ml_history_read((uint8_t *)&h, sizeof(h))) return false;
    FILE *f = fopen(path, "wb");
    if (!f) return false;
    bool ok = fwrite(&h, sizeof(h), 1, f) == 1;
    fclose(f);
    return ok;
}

static bool load_model_file(const char *path) {
    static ai_tuning_history_t h;
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    bool ok = fread(&h, sizeof(h), 1, f) == 1;
    fclose(f);
    return ok && flash_ml_history_write((const uint8_t *)&h, sizeof(h));
}

static void usage(const char *argv0) {
    printf("Usage: %s [options]\n"
           "  --target <gn>        Characterization / charge target (default 43.5)\n"
           "  --charges <n>        Normal charges to throw afterwards (default 5)\n"
           "  --seed <n>           RNG seed (default 1)\n"
           "  --kernel <gn>        Powder kernel weight (default from plant)\n"
           "  --coarse-tail <gn>   Simulated coarse tail\n"
           "  --fine-tail <gn>     Simulated fine tail\n"
           "  --noise <frac>       Flow noise fraction\n"
           "  --coarse-k <gps>     Coarse flow at 1 rps\n"
           "  --fine-k <gps>       Fine flow at 1 rps\n"
           "  --no-tare            Leave scale_handle->force_zero unset (no auto tare)\n"
           "  --stall-s <s>        Watchdog limit per phase (default 60)\n"
           "  --save-model <file>  Write the fitted model to a file after characterization\n"
           "  --load-model <file>  Skip characterization; throw charges with a saved model\n"
           "  --adaptive           Throw the charges with the legacy adaptive controller\n"
           "                       (uses the AI model) instead of the default PID\n"
           "  -q / -v / -vv        Verbosity\n", argv0);
}

static int run(int argc, char **argv) {
    float target = 43.5f;
    int charges = 5;
    uint64_t seed = 1;
    bool tare = true;
    const char *save_path = NULL;
    const char *load_path = NULL;
    bool adaptive = false;

    sim_plant_defaults(&g_plant_cfg);
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--target") && i + 1 < argc) target = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--charges") && i + 1 < argc) charges = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--seed") && i + 1 < argc) seed = strtoull(argv[++i], NULL, 10);
        else if (!strcmp(argv[i], "--kernel") && i + 1 < argc) g_plant_cfg.kernel_weight_gn = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--coarse-tail") && i + 1 < argc) g_plant_cfg.coarse_tail_gn = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--fine-tail") && i + 1 < argc) g_plant_cfg.fine_tail_gn = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--noise") && i + 1 < argc) g_plant_cfg.flow_noise_frac = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--coarse-k") && i + 1 < argc) g_plant_cfg.coarse_flow_k = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--fine-k") && i + 1 < argc) g_plant_cfg.fine_flow_k = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--no-tare")) tare = false;
        else if (!strcmp(argv[i], "--stall-s") && i + 1 < argc) g_stall_s = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--save-model") && i + 1 < argc) save_path = argv[++i];
        else if (!strcmp(argv[i], "--load-model") && i + 1 < argc) load_path = argv[++i];
        else if (!strcmp(argv[i], "--adaptive")) adaptive = true;
        else if (!strcmp(argv[i], "-q")) g_sim_verbosity = 0;
        else if (!strcmp(argv[i], "-v")) g_sim_verbosity = 1;
        else if (!strcmp(argv[i], "-vv")) g_sim_verbosity = 2;
        else { usage(argv[0]); return 1; }
    }
    setvbuf(stdout, NULL, _IOLBF, 0);

    sim_plant_reset(&g_plant_cfg, &g_plant_state, seed);
    sim_rt_start();

    setup_profile();
    charge_mode_config_init();
    if (load_path != NULL && !load_model_file(load_path)) {
        printf("could not load model from %s\n", load_path);
        return 1;
    }
    ai_tuning_init();
    if (tare) {
        g_sim_scale_handle.force_zero = sim_force_zero;
        scale_config.scale_handle = &g_sim_scale_handle;
    }

    printf("\n=== AI characterization: target %.2f gn, seed %llu%s ===\n",
           target, (unsigned long long)seed, tare ? "" : ", no auto tare");
    printf("  plant: coarse k %.2f gn/s/rps, fine k %.3f, coarse tail %.3f, fine tail %.3f, kernel %.4f, noise %.0f%%\n",
           g_plant_cfg.coarse_flow_k, g_plant_cfg.fine_flow_k, g_plant_cfg.coarse_tail_gn,
           g_plant_cfg.fine_tail_gn, g_plant_cfg.kernel_weight_gn, g_plant_cfg.flow_noise_frac * 100.0f);

    std::thread(watchdog_thread).detach();

    // The charges after characterization use the PID controller unless the
    // legacy adaptive controller (the only thing that drives charges from the
    // AI model) is switched on.
    charge_mode_config.eeprom_charge_mode_data.use_adaptive_controller = adaptive;

    if (load_path != NULL) {
        print_model();
        run_charges(target, charges);
        return 0;
    }

    charge_mode_config.target_charge_weight = target;
    if (!ai_tuning_start(profile_get_selected(), target)) {
        printf("ai_tuning_start() failed\n");
        return 3;
    }

    TickType_t t0 = xTaskGetTickCount();
    bool complete = run_characterization();
    printf("\n  characterization %s after %.0f s\n",
           complete ? "complete" : "did NOT complete", (xTaskGetTickCount() - t0) / 1000.0);
    print_session("  ");
    if (!complete) return 3;

    if (!ai_tuning_apply_params()) {
        printf("  ai_tuning_apply_params() failed\n");
        return 3;
    }
    print_model();
    if (save_path != NULL) {
        printf("  model %s %s\n", save_model_file(save_path) ? "saved to" : "could NOT be saved to", save_path);
    }
    run_charges(target, charges);
    return 0;
}

int main(int argc, char **argv) {
    int rc = run(argc, argv);
    sim_rt_stop();   // join the plant ticker so exit doesn't std::terminate
    return rc;
}

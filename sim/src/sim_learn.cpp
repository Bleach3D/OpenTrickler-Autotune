// Learn-from-every-throw prototype harness for the OpenTrickler simulator.
//
// Throws charges with the firmware's real PID charge loop. After each one it
// hands what the charge loop saw (scale readings at the coarse stop, ~1.5 s
// later, at the final stop, and settled) to throw_learner, which adjusts the
// coarse and fine stop thresholds used by the next charge.
//
// Run with --fixed to keep the thresholds where they start, for comparison.
// --change-at N alters the powder flow from charge N on, to check that the
// learning follows a change instead of overthrowing through it.

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>

#include <FreeRTOS.h>
#include <task.h>

#include "charge_mode.h"
#include "profile.h"
#include "ai_tuning.h"
#include "motors.h"

#include "sim_plant.h"
#include "sim_harness.h"
#include "throw_learner.h"

extern "C" {
#include "scale.h"
void sim_rt_start(void);
void sim_rt_stop(void);
}

extern sim_plant_config_t g_plant_cfg;
extern sim_plant_state_t g_plant_state;
extern charge_mode_config_t charge_mode_config;
std::recursive_mutex &sim_plant_mutex();

extern void charge_mode_wait_for_complete(void);
extern void charge_mode_stabilize(void);

extern "C" { int g_sim_verbosity = 1; }

// ---------------------------------------------------------------------------
// What the charge loop would record, captured from motor commands. The
// firmware would take these from its own readings at the same moments.
// ---------------------------------------------------------------------------
static float g_last_coarse_rps = 0.0f;
static float g_last_fine_rps = 0.0f;
static bool g_coarse_ran = false;
static TickType_t g_coarse_stop_tick = 0;
static float g_coarse_stop_reading = NAN;
static float g_after_coarse_reading = NAN;
static float g_fine_stop_reading = NAN;
static TickType_t g_fine_stop_tick = 0;
static const TickType_t kAfterCoarseMs = 1500;

static float reading_now(void) { return (float)g_plant_state.reported_weight_gn; }

extern "C" void sim_harness_on_motor_command(motor_select_t motor, float rps) {
    TickType_t now = xTaskGetTickCount();
    if (motor == SELECT_COARSE_TRICKLER_MOTOR) {
        if (rps > 0.0f) g_coarse_ran = true;
        if (rps <= 0.0f && g_last_coarse_rps > 0.0f && g_coarse_stop_tick == 0) {
            g_coarse_stop_tick = now;
            g_coarse_stop_reading = reading_now();
        }
        g_last_coarse_rps = rps;
    } else if (motor == SELECT_FINE_TRICKLER_MOTOR) {
        if (rps <= 0.0f && g_last_fine_rps > 0.0f) {
            g_fine_stop_reading = reading_now();   // last one wins: the final stop
            g_fine_stop_tick = now;
        }
        g_last_fine_rps = rps;
    }
    if (g_coarse_stop_tick != 0 && !std::isfinite(g_after_coarse_reading) &&
        now - g_coarse_stop_tick >= kAfterCoarseMs) {
        g_after_coarse_reading = reading_now();
    }
    if (g_sim_verbosity >= 3) {
        printf("      [%7u ms] %s -> %.3f rps  (pan %.3f, reading %.3f)\n", (unsigned)now,
               motor == SELECT_COARSE_TRICKLER_MOTOR ? "coarse" : "fine  ", rps,
               g_plant_state.true_weight_gn, reading_now());
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

static void usage(const char *argv0) {
    printf("Usage: %s [options]\n"
           "  --target <gn>        Charge weight (default 43.5)\n"
           "  --charges <n>        Number of charges (default 25)\n"
           "  --coarse-stop <gn>   Starting coarse stop threshold (default 4.0, firmware default)\n"
           "  --fine-stop <gn>     Starting fine stop threshold (default 0.03, firmware default)\n"
           "  --reserve <gn>       What coarse should leave for the fine tube (default 0.6)\n"
           "  --fixed              Don't learn; keep the starting thresholds\n"
           "  --change-at <n>      From charge n, scale coarse flow by --change-flow and\n"
           "                       coarse/fine tail by --change-tail (defaults 1.3 / 1.5)\n"
           "  --change-flow <x> / --change-tail <x>\n"
           "  --fine-max <rps>     Profile fine max speed (default 2.2)\n"
           "  --coarse-kp <x> / --fine-kp <x>  Profile gains (default 0.35 / 3.0)\n"
           "  --seed <n>, --kernel <gn>, --coarse-tail <gn>, --fine-tail <gn>, --noise <frac>\n"
           "  --drift <gn/s>       Scale zero drift (default from plant)\n"
           "  -q / -v / -vvv       Summary / per charge / motor trace\n", argv0);
}

static int run(int argc, char **argv) {
    float target = 43.5f;
    int charges = 25;
    uint64_t seed = 1;
    float coarse_stop = 4.0f, fine_stop = 0.03f, reserve = 0.6f;
    bool learn = true;
    int change_at = -1;
    float change_flow = 1.3f, change_tail = 1.5f;
    float fine_max = 2.2f;
    float coarse_kp = 0.35f, fine_kp = 3.0f;

    sim_plant_defaults(&g_plant_cfg);
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        bool v = i + 1 < argc;
        if (!strcmp(a, "--target") && v) target = strtof(argv[++i], NULL);
        else if (!strcmp(a, "--charges") && v) charges = atoi(argv[++i]);
        else if (!strcmp(a, "--coarse-stop") && v) coarse_stop = strtof(argv[++i], NULL);
        else if (!strcmp(a, "--fine-stop") && v) fine_stop = strtof(argv[++i], NULL);
        else if (!strcmp(a, "--reserve") && v) reserve = strtof(argv[++i], NULL);
        else if (!strcmp(a, "--fixed")) learn = false;
        else if (!strcmp(a, "--change-at") && v) change_at = atoi(argv[++i]);
        else if (!strcmp(a, "--change-flow") && v) change_flow = strtof(argv[++i], NULL);
        else if (!strcmp(a, "--change-tail") && v) change_tail = strtof(argv[++i], NULL);
        else if (!strcmp(a, "--fine-max") && v) fine_max = strtof(argv[++i], NULL);
        else if (!strcmp(a, "--coarse-kp") && v) coarse_kp = strtof(argv[++i], NULL);
        else if (!strcmp(a, "--fine-kp") && v) fine_kp = strtof(argv[++i], NULL);
        else if (!strcmp(a, "--seed") && v) seed = strtoull(argv[++i], NULL, 10);
        else if (!strcmp(a, "--kernel") && v) g_plant_cfg.kernel_weight_gn = strtof(argv[++i], NULL);
        else if (!strcmp(a, "--coarse-tail") && v) g_plant_cfg.coarse_tail_gn = strtof(argv[++i], NULL);
        else if (!strcmp(a, "--fine-tail") && v) g_plant_cfg.fine_tail_gn = strtof(argv[++i], NULL);
        else if (!strcmp(a, "--noise") && v) g_plant_cfg.flow_noise_frac = strtof(argv[++i], NULL);
        else if (!strcmp(a, "--drift") && v) g_plant_cfg.scale_drift_gn_per_s = strtof(argv[++i], NULL);
        else if (!strcmp(a, "-q")) g_sim_verbosity = 0;
        else if (!strcmp(a, "-v")) g_sim_verbosity = 1;
        else if (!strcmp(a, "-vvv")) g_sim_verbosity = 3;
        else { usage(argv[0]); return 1; }
    }
    setvbuf(stdout, NULL, _IOLBF, 0);

    sim_plant_reset(&g_plant_cfg, &g_plant_state, seed);
    sim_rt_start();
    setup_profile();
    profile_get_selected()->fine_max_flow_speed_rps = fine_max;
    profile_get_selected()->coarse_kp = coarse_kp;
    profile_get_selected()->fine_kp = fine_kp;
    charge_mode_config_init();
    ai_tuning_init();
    charge_mode_config.eeprom_charge_mode_data.use_adaptive_controller = false;

    float tol = charge_mode_config.eeprom_charge_mode_data.accept_tolerance_gn;
    throw_learner_t learner;
    throw_learner_init(&learner, coarse_stop, fine_stop, reserve, tol);

    printf("\n=== %s: %d charges at %.2f gn, start coarse stop %.2f / fine stop %.3f, reserve %.2f ===\n",
           learn ? "Learning thresholds" : "Fixed thresholds", charges, target,
           learner.coarse_stop_gn, learner.fine_stop_gn, reserve);
    printf("  profile: coarse kp %.2f, fine kp %.2f, fine max %.2f rps\n", coarse_kp, fine_kp, fine_max);
    printf("  plant: coarse k %.2f, coarse tail %.2f, fine tail %.3f, kernel %.3f, drift %.4f gn/s; tolerance +/-%.4f\n",
           g_plant_cfg.coarse_flow_k, g_plant_cfg.coarse_tail_gn, g_plant_cfg.fine_tail_gn,
           g_plant_cfg.kernel_weight_gn, g_plant_cfg.scale_drift_gn_per_s, tol);
    if (g_sim_verbosity >= 1) {
        printf("   #  coarse   fine  |   error  result |  total  coarse   fine | left@coarse fine tail\n");
    }

    int over = 0, within = 0, under = 0;
    int over_after_settle = 0;
    double abs_err_sum = 0, time_sum_last = 0;
    int last_n = 0;
    const int settle_from = charges > 10 ? charges - 10 : 0;

    for (int i = 0; i < charges; i++) {
        if (i == change_at) {
            g_plant_cfg.coarse_flow_k *= change_flow;
            g_plant_cfg.coarse_tail_gn *= change_tail;
            g_plant_cfg.fine_tail_gn *= change_tail;
            printf("  --- powder change: coarse flow x%.2f, tails x%.2f ---\n", change_flow, change_tail);
        }
        {
            std::lock_guard<std::recursive_mutex> lock(sim_plant_mutex());
            sim_plant_empty_pan(&g_plant_state);
            sim_plant_zero(&g_plant_state);
        }
        vTaskDelay(500);

        float used_coarse = learner.coarse_stop_gn, used_fine = learner.fine_stop_gn;
        charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold = used_coarse;
        charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold = used_fine;

        g_coarse_ran = false;
        g_coarse_stop_tick = 0;
        g_fine_stop_tick = 0;
        g_coarse_stop_reading = g_after_coarse_reading = g_fine_stop_reading = NAN;
        g_last_coarse_rps = g_last_fine_rps = 0.0f;

        charge_mode_config.target_charge_weight = target;
        charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_COMPLETE;
        TickType_t start = xTaskGetTickCount();
        charge_mode_wait_for_complete();
        TickType_t end = xTaskGetTickCount();
        charge_mode_stabilize();
        vTaskDelay(1500);
        float settled = reading_now();
        double true_err = g_plant_state.true_weight_gn - target;

        throw_observation_t obs = {};
        obs.target_gn = target;
        obs.coarse_ran = g_coarse_ran;
        obs.coarse_stop_reading_gn = g_coarse_stop_reading;
        // The after-coarse reading only counts if it was taken before the
        // final stop; otherwise the charge was already over.
        obs.after_coarse_reading_gn =
            (g_fine_stop_tick != 0 && g_coarse_stop_tick != 0 &&
             g_fine_stop_tick - g_coarse_stop_tick < kAfterCoarseMs) ? NAN : g_after_coarse_reading;
        obs.fine_stop_reading_gn = g_fine_stop_reading;
        obs.final_reading_gn = settled;

        throw_learner_result_t res = {};
        bool used = learn && throw_learner_update(&learner, &obs, &res);
        if (!learn) {
            res.fine_tail_gn = settled - g_fine_stop_reading;
            res.remaining_after_coarse_gn = target - g_after_coarse_reading;
        }

        const char *verdict = true_err > tol ? "OVER" : (true_err < -tol ? "under" : "ok");
        if (true_err > tol) { over++; if (i >= settle_from) over_after_settle++; }
        else if (true_err < -tol) under++;
        else within++;
        abs_err_sum += fabs(true_err);
        double secs = (end - start) / 1000.0;
        double coarse_s = g_coarse_stop_tick > start ? (g_coarse_stop_tick - start) / 1000.0 : 0.0;
        if (i >= settle_from) { time_sum_last += secs; last_n++; }

        if (g_sim_verbosity >= 1) {
            printf("  %2d  %5.2f  %5.3f  | %+7.3f  %-5s  | %5.1fs  %5.1fs %5.1fs |   %6.3f     %6.3f %s%s%s\n",
                   i + 1, used_coarse, used_fine, true_err, verdict, secs, coarse_s, secs - coarse_s,
                   res.remaining_after_coarse_gn, res.fine_tail_gn,
                   (learn && !used) ? " (not learned)" : "",
                   res.coarse_backoff ? " coarse-backoff" : "",
                   res.fine_backoff ? " fine-backoff" : "");
        }
    }

    printf("\n  overthrows %d (in last %d: %d), within tolerance %d, under %d, of %d\n",
           over, charges - settle_from, over_after_settle, within, under, charges);
    printf("  mean |error| %.3f gn; mean time over last %d charges %.1f s\n",
           abs_err_sum / charges, last_n, last_n ? time_sum_last / last_n : 0.0);
    if (learn) {
        printf("  learned: coarse stop %.2f gn, fine stop %.3f gn\n",
               learner.coarse_stop_gn, learner.fine_stop_gn);
    }
    return 0;
}

int main(int argc, char **argv) {
    int rc = run(argc, argv);
    sim_rt_stop();
    return rc;
}

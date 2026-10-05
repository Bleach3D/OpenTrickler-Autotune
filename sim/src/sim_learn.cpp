// Learn From Throws harness for the OpenTrickler simulator.
//
// Throws charges with the firmware's real PID charge loop with Learn From
// Throws switched on for the profile, exactly as the web GUI does. Everything
// that learns -- the observation capture in charge_mode.cpp, the update in
// charge_mode_stabilize(), src/learn_from_throws.c and src/throw_learner.c --
// is the firmware code; this file only drives charges and prints results.
//
// Run with --fixed to keep the thresholds where they start (learning off),
// for comparison. --change-at N alters the powder flow from charge N on, to
// check that the learning follows a change instead of overthrowing through it.

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
#include "learn_from_throws.h"

#include "sim_plant.h"
#include "sim_harness.h"

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

// Motor commands are only used for the coarse/fine time split.
static float g_last_coarse_rps = 0.0f;
static TickType_t g_coarse_stop_tick = 0;

extern "C" void sim_harness_on_motor_command(motor_select_t motor, float rps) {
    TickType_t now = xTaskGetTickCount();
    if (motor == SELECT_COARSE_TRICKLER_MOTOR) {
        if (rps <= 0.0f && g_last_coarse_rps > 0.0f && g_coarse_stop_tick == 0) {
            g_coarse_stop_tick = now;
        }
        g_last_coarse_rps = rps;
    }
    if (g_sim_verbosity >= 3) {
        printf("      [%7u ms] %s -> %.3f rps  (pan %.3f, reading %.3f)\n", (unsigned)now,
               motor == SELECT_COARSE_TRICKLER_MOTOR ? "coarse" : "fine  ", rps,
               g_plant_state.true_weight_gn, g_plant_state.reported_weight_gn);
    }
}

static void setup_profile(float coarse_kp, float fine_kp, float fine_max) {
    profile_data_init();
    profile_t *p = profile_get_selected();
    if (p == NULL) return;
    snprintf(p->name, sizeof(p->name), "SimProfile");
    p->coarse_kp = coarse_kp; p->coarse_ki = 0.0f; p->coarse_kd = 0.0f;
    p->fine_kp = fine_kp;     p->fine_ki = 0.0f;   p->fine_kd = 0.0f;
    p->coarse_min_flow_speed_rps = 0.30f;
    p->coarse_max_flow_speed_rps = 8.0f;
    p->fine_min_flow_speed_rps = 0.08f;
    p->fine_max_flow_speed_rps = fine_max;
}

static void usage(const char *argv0) {
    printf("Usage: %s [options]\n"
           "  --target <gn>        Charge weight (default 43.5)\n"
           "  --charges <n>        Number of charges (default 25)\n"
           "  --coarse-stop <gn>   Profile coarse stop threshold to start from (default 4.0, firmware default)\n"
           "  --fine-stop <gn>     Profile fine stop threshold to start from (default 0.03, firmware default)\n"
           "  --reserve <gn>       What coarse should leave for the fine tube (default 0.6)\n"
           "  --fixed              Learning off; keep the profile's thresholds\n"
           "  --change-at <n>      From charge n, scale coarse flow by --change-flow and\n"
           "                       coarse/fine tail by --change-tail (defaults 1.3 / 1.5)\n"
           "  --change-flow <x> / --change-tail <x>\n"
           "  --fine-max <rps>     Profile fine max speed (default 2.2)\n"
           "  --coarse-kp <x> / --fine-kp <x>  Profile gains (default 0.35 / 3.0)\n"
           "  --seed <n>, --kernel <gn>, --coarse-tail <gn>, --fine-tail <gn>, --noise <frac>\n"
           "  --drift <gn/s>       Scale zero drift (default from plant)\n"
           "  -q / -v / -vvv       Summary / per charge / motor trace\n", argv0);
}

static const char *fmt(char *buf, size_t len, float v, const char *f) {
    if (!std::isfinite(v)) snprintf(buf, len, "%s", "   -  ");
    else snprintf(buf, len, f, v);
    return buf;
}

static int run(int argc, char **argv) {
    float target = 43.5f;
    int charges = 25;
    uint64_t seed = 1;
    float coarse_stop = 4.0f, fine_stop = 0.03f, reserve = LEARN_FROM_THROWS_DEFAULT_RESERVE;
    bool learn = true;
    int change_at = -1;
    float change_flow = 1.3f, change_tail = 1.5f;
    float fine_max = 2.2f, coarse_kp = 0.35f, fine_kp = 3.0f;

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
    setup_profile(coarse_kp, fine_kp, fine_max);
    charge_mode_config_init();
    ai_tuning_init();
    charge_mode_config.eeprom_charge_mode_data.use_adaptive_controller = false;
    // The profile's own thresholds: what PID uses with learning off, and
    // where learning starts from when it is switched on.
    charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold = coarse_stop;
    charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold = fine_stop;

    // Same calls the web GUI's Learn From Throws switch makes.
    learn_from_throws_init();
    const uint8_t pf = (uint8_t)profile_get_selected_idx();
    learn_from_throws_reset(pf);
    learn_from_throws_set_reserve(pf, reserve);
    learn_from_throws_set_enabled(pf, learn);

    float tol = charge_mode_config.eeprom_charge_mode_data.accept_tolerance_gn;
    printf("\n=== %s: %d charges at %.2f gn, start coarse stop %.2f / fine stop %.3f, reserve %.2f ===\n",
           learn ? "Learn From Throws on" : "Learn From Throws off (fixed thresholds)", charges, target,
           coarse_stop, fine_stop, reserve);
    printf("  profile: coarse kp %.2f, fine kp %.2f, fine max %.2f rps\n", coarse_kp, fine_kp, fine_max);
    printf("  plant: coarse k %.2f, coarse tail %.2f, fine tail %.3f, kernel %.3f, drift %.4f gn/s; tolerance +/-%.4f\n",
           g_plant_cfg.coarse_flow_k, g_plant_cfg.coarse_tail_gn, g_plant_cfg.fine_tail_gn,
           g_plant_cfg.kernel_weight_gn, g_plant_cfg.scale_drift_gn_per_s, tol);
    if (g_sim_verbosity >= 1) {
        printf("   #  coarse   fine  |   error  result |  total  coarse   fine | left@coarse fine tail\n");
    }

    int over = 0, within = 0, under = 0, over_last = 0;
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

        float used_coarse = coarse_stop, used_fine = fine_stop;
        learn_from_throws_thresholds(pf, &used_coarse, &used_fine);
        g_coarse_stop_tick = 0;
        g_last_coarse_rps = 0.0f;

        charge_mode_config.target_charge_weight = target;
        charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_COMPLETE;
        TickType_t start = xTaskGetTickCount();
        charge_mode_wait_for_complete();
        TickType_t end = xTaskGetTickCount();
        double true_before_stabilize = g_plant_state.true_weight_gn;
        charge_mode_stabilize();   // learning happens in here, before any top-up
        vTaskDelay(1500);
        (void)true_before_stabilize;
        double true_err = g_plant_state.true_weight_gn - target;

        learn_from_throws_last_t last;
        bool have_last = learn && learn_from_throws_get_last(&last);

        const char *verdict = true_err > tol ? "OVER" : (true_err < -tol ? "under" : "ok");
        if (true_err > tol) { over++; if (i >= settle_from) over_last++; }
        else if (true_err < -tol) under++;
        else within++;
        abs_err_sum += fabs(true_err);
        double secs = (end - start) / 1000.0;
        double coarse_s = g_coarse_stop_tick > start ? (g_coarse_stop_tick - start) / 1000.0 : 0.0;
        if (i >= settle_from) { time_sum_last += secs; last_n++; }

        if (g_sim_verbosity >= 1) {
            char b1[16], b2[16];
            printf("  %2d  %5.2f  %5.3f  | %+7.3f  %-5s  | %5.1fs  %5.1fs %5.1fs |   %s     %s %s%s\n",
                   i + 1, used_coarse, used_fine, true_err, verdict, secs, coarse_s, secs - coarse_s,
                   fmt(b1, sizeof(b1), have_last ? last.result.remaining_after_coarse_gn : NAN, "%6.3f"),
                   fmt(b2, sizeof(b2), have_last ? last.result.fine_tail_gn : NAN, "%6.3f"),
                   have_last && last.result.coarse_backoff ? " coarse-backoff" : "",
                   have_last && last.result.fine_backoff ? " fine-backoff" : "");
        }
    }

    printf("\n  overthrows %d (in last %d: %d), within tolerance %d, under %d, of %d\n",
           over, charges - settle_from, over_last, within, under, charges);
    printf("  mean |error| %.3f gn; mean time over last %d charges %.1f s\n",
           abs_err_sum / charges, last_n, last_n ? time_sum_last / last_n : 0.0);
    if (learn) {
        learn_from_throws_profile_t p;
        learn_from_throws_get(pf, &p);
        printf("  learned: coarse stop %.2f gn, fine stop %.3f gn after %u charges\n",
               p.learner.coarse_stop_gn, p.learner.fine_stop_gn, (unsigned)p.learner.charges_learned);
    }
    return 0;
}

int main(int argc, char **argv) {
    int rc = run(argc, argv);
    sim_rt_stop();
    return rc;
}

// OpenTrickler simulator harness.
//
// Runs the firmware's real charge controller against the simulated plant,
// repeatedly, and reports the statistics that actually matter for reloading:
// mean error, standard deviation, spread, and time per throw.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>

#include "charge_mode.h"
#include "profile.h"
#include "ai_tuning.h"
#include "motors.h"
#include "eeprom.h"

#include "sim_plant.h"
#include "sim_harness.h"

extern "C" {
#include "scale.h"
void sim_rt_start(void);
void sim_rt_stop(void);
}

extern sim_plant_config_t g_plant_cfg;
extern sim_plant_state_t g_plant_state;

// Phase functions from charge_mode.cpp.
// charge_mode.cpp defines these; the header does not export the config
// global, so declare it here. Global variables are not name-mangled, so a
// plain extern links against the C++ translation unit correctly.
extern charge_mode_config_t charge_mode_config;
extern void charge_mode_wait_for_complete(void);
extern void charge_mode_stabilize(void);

extern "C" { int g_sim_verbosity = 1; }

// ---------------------------------------------------------------------------
// Motor trace
// ---------------------------------------------------------------------------
static float g_last_coarse_rps = -1.0f;
static float g_last_fine_rps = -1.0f;
static TickType_t g_coarse_stop_tick = 0;
static bool g_coarse_ran = false;
static double g_coarse_stop_weight = 0.0;

extern "C" void sim_harness_on_motor_command(motor_select_t motor, float rps) {
    if (motor == SELECT_COARSE_TRICKLER_MOTOR) {
        if (rps > 0.0f && g_last_coarse_rps <= 0.0f) {
            g_coarse_ran = true;
        }
        // Record the moment coarse hands off: this is the number the
        // "coarse stop threshold" setting is really about.
        if (rps <= 0.0f && g_last_coarse_rps > 0.0f) {
            g_coarse_stop_tick = xTaskGetTickCount();
            g_coarse_stop_weight = g_plant_state.true_weight_gn;
        }
        if (g_sim_verbosity >= 2 && fabsf(rps - g_last_coarse_rps) > 0.001f) {
            printf("      [%7u ms] coarse -> %.3f rps  (pan %.3f gn)\n",
                   (unsigned)xTaskGetTickCount(), rps, g_plant_state.true_weight_gn);
        }
        g_last_coarse_rps = rps;
    } else if (motor == SELECT_FINE_TRICKLER_MOTOR) {
        if (g_sim_verbosity >= 2 && fabsf(rps - g_last_fine_rps) > 0.001f) {
            printf("      [%7u ms] fine   -> %.3f rps  (pan %.3f gn)\n",
                   (unsigned)xTaskGetTickCount(), rps, g_plant_state.true_weight_gn);
        }
        g_last_fine_rps = rps;
    }
}

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------
typedef struct {
    double error_gn;
    double elapsed_s;
    double coarse_handoff_remaining_gn;
    double fine_phase_s;
} charge_result_t;

static int compare_double(const void *a, const void *b) {
    double da = *(const double *)a;
    double db = *(const double *)b;
    return (da > db) - (da < db);
}

static void print_stats(const char *label, double *values, int n, const char *unit) {
    if (n <= 0) {
        return;
    }
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        sum += values[i];
    }
    double mean = sum / n;
    double var = 0.0;
    for (int i = 0; i < n; i++) {
        double d = values[i] - mean;
        var += d * d;
    }
    // Sample standard deviation (n-1), which is what you would compute from
    // a string of measured charges.
    double sd = (n > 1) ? sqrt(var / (n - 1)) : 0.0;

    double *sorted = (double *)malloc(sizeof(double) * (size_t)n);
    memcpy(sorted, values, sizeof(double) * (size_t)n);
    qsort(sorted, (size_t)n, sizeof(double), compare_double);
    double min = sorted[0];
    double max = sorted[n - 1];
    double median = (n % 2) ? sorted[n / 2] : 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]);
    free(sorted);

    printf("  %-26s mean %+8.4f  sd %7.4f  min %+8.4f  med %+8.4f  max %+8.4f %s\n",
           label, mean, sd, min, median, max, unit);
}

// ---------------------------------------------------------------------------
// One charge
// ---------------------------------------------------------------------------
static charge_result_t run_one_charge(float target_gn) {
    charge_result_t r;
    memset(&r, 0, sizeof(r));

    sim_plant_empty_pan(&g_plant_state);
    g_coarse_ran = false;
    g_coarse_stop_tick = 0;
    g_coarse_stop_weight = 0.0;
    g_last_coarse_rps = -1.0f;
    g_last_fine_rps = -1.0f;

    charge_mode_config.target_charge_weight = target_gn;
    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_COMPLETE;

    TickType_t start = xTaskGetTickCount();
    charge_mode_wait_for_complete();
    charge_mode_stabilize();
    TickType_t end = xTaskGetTickCount();

    // Let everything settle so the recorded value matches what the operator
    // would actually read off the scale.
    sim_advance_ticks(1500);
    float settled = 0.0f;
    scale_block_wait_for_next_measurement(500, &settled);

    r.error_gn = (double)g_plant_state.true_weight_gn - (double)target_gn;
    r.elapsed_s = (double)(end - start) / 1000.0;
    r.coarse_handoff_remaining_gn = g_coarse_ran
        ? ((double)target_gn - g_coarse_stop_weight)
        : NAN;
    r.fine_phase_s = g_coarse_ran
        ? (double)(end - g_coarse_stop_tick) / 1000.0
        : r.elapsed_s;

    return r;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
static void setup_profile(void) {
    profile_data_init();
    profile_t *p = profile_get_selected();
    if (p == NULL) {
        return;
    }
    snprintf(p->name, sizeof(p->name), "SimProfile");
    p->coarse_kp = 0.35f;
    p->coarse_ki = 0.0f;
    p->coarse_kd = 0.0f;
    p->fine_kp = 3.0f;
    p->fine_ki = 0.0f;
    p->fine_kd = 0.0f;
    p->coarse_min_flow_speed_rps = 0.30f;
    p->coarse_max_flow_speed_rps = 8.0f;
    p->fine_min_flow_speed_rps = 0.08f;
    p->fine_max_flow_speed_rps = 2.20f;
}

static void usage(const char *argv0) {
    printf("OpenTrickler simulator\n\n");
    printf("Usage: %s [options]\n\n", argv0);
    printf("  --target <gn>        Target charge weight (default 45.0)\n");
    printf("  --charges <n>        Number of charges to run (default 20)\n");
    printf("  --seed <n>           RNG seed for repeatable runs (default 1)\n");
    printf("  --kernel <gn>        Powder kernel weight (default 0.04, e.g. N565)\n");
    printf("  --coarse-stop <gn>   Coarse stop threshold setting (default 0.80)\n");
    printf("  --coarse-auto        Let AI tuning choose the coarse handoff\n");
    printf("  --bias <0..1>        Accuracy/speed bias, 0=fast 1=accurate (default 0.5)\n");
    printf("  --coarse-tail <gn>   Simulated coarse tail (default 0.85)\n");
    printf("  --fine-tail <gn>     Simulated fine tail (default 0.035)\n");
    printf("  --noise <frac>       Flow noise fraction (default 0.10)\n");
    printf("  -v                   Per-charge output (default)\n");
    printf("  -vv                  Full motor trace\n");
    printf("  -q                   Summary only\n");
}

int main(int argc, char **argv) {
    float target = 45.0f;
    int charges = 20;
    uint64_t seed = 1;
    float coarse_stop = 0.80f;
    bool coarse_auto = false;
    float bias = 0.5f;

    sim_plant_defaults(&g_plant_cfg);

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--target") && i + 1 < argc) target = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--charges") && i + 1 < argc) charges = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--seed") && i + 1 < argc) seed = (uint64_t)strtoull(argv[++i], NULL, 10);
        else if (!strcmp(argv[i], "--kernel") && i + 1 < argc) g_plant_cfg.kernel_weight_gn = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--coarse-stop") && i + 1 < argc) coarse_stop = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--coarse-auto")) coarse_auto = true;
        else if (!strcmp(argv[i], "--bias") && i + 1 < argc) bias = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--coarse-tail") && i + 1 < argc) g_plant_cfg.coarse_tail_gn = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--fine-tail") && i + 1 < argc) g_plant_cfg.fine_tail_gn = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--noise") && i + 1 < argc) g_plant_cfg.flow_noise_frac = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "-v")) g_sim_verbosity = 1;
        else if (!strcmp(argv[i], "-vv")) g_sim_verbosity = 2;
        else if (!strcmp(argv[i], "-q")) g_sim_verbosity = 0;
        else { usage(argv[0]); return 1; }
    }

    sim_plant_reset(&g_plant_cfg, &g_plant_state, seed);
    // Starts the plant ticker thread; without it the pan never fills and the
    // first charge waits forever.
    sim_rt_start();

    setup_profile();
    charge_mode_config_init();
    ai_tuning_init();

    charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold = coarse_stop;
    charge_mode_config.eeprom_charge_mode_data.ai_coarse_stop_auto = coarse_auto;
    charge_mode_config.eeprom_charge_mode_data.ai_accuracy_speed_bias = bias;

    printf("\n=== OpenTrickler simulation ===\n");
    printf("  target %.2f gn | %d charges | seed %llu\n",
           target, charges, (unsigned long long)seed);
    printf("  kernel %.4f gn | coarse tail %.3f gn | fine tail %.3f gn | flow noise %.0f%%\n",
           g_plant_cfg.kernel_weight_gn, g_plant_cfg.coarse_tail_gn,
           g_plant_cfg.fine_tail_gn, g_plant_cfg.flow_noise_frac * 100.0f);
    printf("  scale: A&D FX-120i model, %.4f gn resolution, %u ms stream\n",
           g_plant_cfg.scale_resolution_gn, (unsigned)g_plant_cfg.scale_period_ms);
    printf("  coarse stop %.2f gn (%s) | accuracy/speed bias %.2f\n\n",
           coarse_stop, coarse_auto ? "AI decides" : "user setting", bias);

    double *errors = (double *)malloc(sizeof(double) * (size_t)charges);
    double *times = (double *)malloc(sizeof(double) * (size_t)charges);
    double *handoffs = (double *)malloc(sizeof(double) * (size_t)charges);
    double *fine_times = (double *)malloc(sizeof(double) * (size_t)charges);
    int handoff_count = 0;

    for (int i = 0; i < charges; i++) {
        if (g_sim_verbosity >= 2) {
            printf("  --- charge %d ---\n", i + 1);
        }
        charge_result_t r = run_one_charge(target);
        errors[i] = r.error_gn;
        times[i] = r.elapsed_s;
        fine_times[i] = r.fine_phase_s;
        if (!isnan(r.coarse_handoff_remaining_gn)) {
            handoffs[handoff_count++] = r.coarse_handoff_remaining_gn;
        }
        if (g_sim_verbosity >= 1) {
            printf("  charge %3d: final %8.3f gn  err %+7.3f  total %6.2f s  fine %6.2f s  handoff left %6.3f gn\n",
                   i + 1,
                   (double)g_plant_state.true_weight_gn,
                   r.error_gn, r.elapsed_s, r.fine_phase_s,
                   r.coarse_handoff_remaining_gn);
        }
    }

    printf("\n=== Results over %d charges ===\n", charges);
    print_stats("Error vs target", errors, charges, "gn");
    print_stats("Total time", times, charges, "s");
    print_stats("Fine phase time", fine_times, charges, "s");
    if (handoff_count > 0) {
        print_stats("Left at coarse handoff", handoffs, handoff_count, "gn");
    }

    int over = 0, under = 0, on = 0;
    double tol = charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin;
    if (tol <= 0.0) {
        tol = 0.02;
    }
    for (int i = 0; i < charges; i++) {
        if (errors[i] > tol) over++;
        else if (errors[i] < -tol) under++;
        else on++;
    }
    printf("\n  Within +/-%.3f gn: %d/%d (%.0f%%)   over: %d   under: %d\n",
           tol, on, charges, 100.0 * on / charges, over, under);

    // The kernel floor is a hard physical limit: no controller can do better
    // than landing within one granule of target.
    printf("  Kernel-limited best case sd: ~%.4f gn (one %.4f gn granule)\n\n",
           g_plant_cfg.kernel_weight_gn / sqrt(12.0), g_plant_cfg.kernel_weight_gn);

    free(errors);
    free(times);
    free(handoffs);
    free(fine_times);
    sim_rt_stop();
    return 0;
}

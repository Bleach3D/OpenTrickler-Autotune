#include "sim_plant.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Grains per gram, for converting the A&D's native 0.001 g resolution.
#define GRAINS_PER_GRAM 15.4323584f

// xorshift64* - deterministic so a seed reproduces a run exactly.
static double sim_rand_unit(sim_plant_state_t *st) {
    uint64_t x = st->rng_state;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    st->rng_state = x;
    return (double)((x * 0x2545F4914F6CDD1DULL) >> 11) / (double)(1ULL << 53);
}

// Approximately normal, mean 0, sd 1 (sum of 3 uniforms; adequate here and
// avoids pulling in a full Box-Muller with its edge cases).
static double sim_rand_normal(sim_plant_state_t *st) {
    double s = sim_rand_unit(st) + sim_rand_unit(st) + sim_rand_unit(st);
    return (s - 1.5) * 2.0;
}

void sim_plant_defaults(sim_plant_config_t *cfg) {
    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));

    // Trickler defaults are a plausible mid-size setup. They are NOT
    // measured from a specific machine - see sim/README.md. Calibrate them
    // against your own hardware before trusting absolute timings.
    // k is flow in gn/s at 1 rps; with the exponent this gives roughly
    // 6 gn/s at 8 rps for the coarse tube, which is a realistic large-tube
    // rate. Getting this wrong by an order of magnitude makes the whole
    // simulation meaningless, so sanity-check it against your machine.
    cfg->coarse_flow_k = 0.55f;
    cfg->coarse_flow_exponent = 1.15f;
    cfg->coarse_spinup_ms = 120.0f;
    cfg->coarse_spindown_ms = 90.0f;
    cfg->coarse_tail_gn = 0.85f;

    // ~0.15 gn/s at 2.2 rps for the fine tube.
    cfg->fine_flow_k = 0.065f;
    cfg->fine_flow_exponent = 1.05f;
    cfg->fine_spinup_ms = 70.0f;
    cfg->fine_spindown_ms = 55.0f;
    cfg->fine_tail_gn = 0.035f;

    cfg->kernel_weight_gn = 0.04f;   // e.g. VihtaVuori N565
    cfg->flow_noise_frac = 0.10f;

    // ---- A&D FX-120i ----
    // Readability 0.001 g, which is ~0.0154 gn. This is the single most
    // important scale characteristic: the firmware can never resolve a
    // difference finer than this, no matter what the control loop wants.
    cfg->scale_resolution_gn = 0.001f * GRAINS_PER_GRAM;
    // FX-i series response time is ~1 s to a stable reading in its standard
    // filter setting. Modelled as a first-order lag.
    cfg->scale_response_ms = 320.0f;
    cfg->scale_noise_gn = 0.006f;
    cfg->scale_drift_gn_per_s = 0.0008f;
    // Streaming output interval. The FX-i can stream faster, but ~100 ms is
    // the practical rate seen in trickler use.
    cfg->scale_period_ms = 100u;
    cfg->scale_stable_band_gn = 0.02f;
    cfg->scale_stable_ms = 400u;
}

void sim_plant_reset(const sim_plant_config_t *cfg, sim_plant_state_t *st, uint64_t seed) {
    if (st == NULL) {
        return;
    }
    (void)cfg;
    memset(st, 0, sizeof(*st));
    st->rng_state = seed ? seed : 0x9E3779B97F4A7C15ULL;
    st->stable = true;
}

void sim_plant_set_coarse_rps(sim_plant_state_t *st, float rps) {
    if (st) {
        st->coarse_cmd_rps = (rps > 0.0f) ? rps : 0.0f;
    }
}

void sim_plant_set_fine_rps(sim_plant_state_t *st, float rps) {
    if (st) {
        st->fine_cmd_rps = (rps > 0.0f) ? rps : 0.0f;
    }
}

void sim_plant_zero(sim_plant_state_t *st) {
    if (st == NULL) {
        return;
    }
    // A tare shifts the reference, it does not remove powder.
    st->drift_gn = 0.0;
    st->indicated_weight_gn -= st->true_weight_gn;
    st->reported_weight_gn = 0.0;
    st->true_weight_gn = 0.0;
    st->indicated_weight_gn = 0.0;
}

void sim_plant_empty_pan(sim_plant_state_t *st) {
    if (st == NULL) {
        return;
    }
    st->true_weight_gn = 0.0;
    st->pending_gn = 0.0;
    st->indicated_weight_gn = 0.0;
    st->reported_weight_gn = 0.0;
    st->coarse_flow_gps = 0.0;
    st->fine_flow_gps = 0.0;
    st->stable = true;
    st->stable_accum_ms = 0;
}

static double sim_target_flow(double cmd_rps, double k, double exponent) {
    if (cmd_rps <= 0.0) {
        return 0.0;
    }
    return k * pow(cmd_rps, exponent);
}

static double sim_approach(double current, double target, double dt_ms, double tau_ms) {
    if (tau_ms <= 1.0) {
        return target;
    }
    double alpha = 1.0 - exp(-dt_ms / tau_ms);
    return current + (target - current) * alpha;
}

void sim_plant_step(const sim_plant_config_t *cfg, sim_plant_state_t *st, uint32_t dt_ms) {
    if (cfg == NULL || st == NULL || dt_ms == 0) {
        return;
    }

    const double dt = (double)dt_ms;
    const double dt_s = dt / 1000.0;

    // --- Motor flow response ---
    // Spin-up and spin-down use different time constants: the tube keeps
    // delivering briefly after the motor stops, which is the physical origin
    // of the "tail" the firmware spends so much effort predicting.
    double coarse_target = sim_target_flow(st->coarse_cmd_rps, cfg->coarse_flow_k, cfg->coarse_flow_exponent);
    double fine_target = sim_target_flow(st->fine_cmd_rps, cfg->fine_flow_k, cfg->fine_flow_exponent);

    // Spin-down time constant is derived from the configured tail mass.
    // For exponential decay the total powder still to come is flow * tau, so
    // tau = tail / flow reproduces the requested tail regardless of the flow
    // rate the motor was running at when it was told to stop. That matches
    // the physical picture: the tail is powder already in flight and on the
    // tube lip, i.e. roughly a fixed mass, not a fixed time.
    //
    // tau has to be latched once, from the flow at the moment of the stop.
    // Recomputing it every step from the decaying flow makes tau grow as the
    // flow falls, which drags the tail out over many seconds and roughly
    // triples its mass.
    if (coarse_target > 0.0) {
        st->coarse_stop_tau_ms = 0.0;
    } else if (st->coarse_stop_tau_ms <= 0.0) {
        double tau = cfg->coarse_spindown_ms;
        if (st->coarse_flow_gps > 0.001) {
            tau = (cfg->coarse_tail_gn / st->coarse_flow_gps) * 1000.0;
            if (tau < 10.0) tau = 10.0;
            if (tau > 3000.0) tau = 3000.0;
        }
        st->coarse_stop_tau_ms = tau;
    }
    if (fine_target > 0.0) {
        st->fine_stop_tau_ms = 0.0;
    } else if (st->fine_stop_tau_ms <= 0.0) {
        double tau = cfg->fine_spindown_ms;
        if (st->fine_flow_gps > 0.0001) {
            tau = (cfg->fine_tail_gn / st->fine_flow_gps) * 1000.0;
            if (tau < 5.0) tau = 5.0;
            if (tau > 3000.0) tau = 3000.0;
        }
        st->fine_stop_tau_ms = tau;
    }
    double coarse_tau = (coarse_target > 0.0) ? cfg->coarse_spindown_ms : st->coarse_stop_tau_ms;
    double fine_tau = (fine_target > 0.0) ? cfg->fine_spindown_ms : st->fine_stop_tau_ms;

    st->coarse_flow_gps = sim_approach(st->coarse_flow_gps, coarse_target, dt,
                                       coarse_target > st->coarse_flow_gps ? cfg->coarse_spinup_ms
                                                                           : coarse_tau);
    st->fine_flow_gps = sim_approach(st->fine_flow_gps, fine_target, dt,
                                     fine_target > st->fine_flow_gps ? cfg->fine_spinup_ms
                                                                     : fine_tau);

    // --- Powder delivered this step ---
    double delivered = (st->coarse_flow_gps + st->fine_flow_gps) * dt_s;
    if (delivered > 0.0 && cfg->flow_noise_frac > 0.0f) {
        double noise = 1.0 + sim_rand_normal(st) * cfg->flow_noise_frac;
        if (noise < 0.0) {
            noise = 0.0;
        }
        delivered *= noise;
    }
    // Powder is granular: it leaves the tube in whole kernels. Accumulate
    // continuously and release whole granules, rather than rounding the pan
    // total each step -- rounding the total would silently discard the
    // sub-kernel remainder on every step and no powder would ever build up.
    st->pending_gn += delivered;
    if (cfg->kernel_weight_gn > 0.0001f) {
        double whole = floor(st->pending_gn / (double)cfg->kernel_weight_gn);
        if (whole > 0.0) {
            double mass = whole * (double)cfg->kernel_weight_gn;
            st->true_weight_gn += mass;
            st->pending_gn -= mass;
        }
    } else {
        st->true_weight_gn += st->pending_gn;
        st->pending_gn = 0.0;
    }

    // --- Scale response ---
    st->drift_gn += cfg->scale_drift_gn_per_s * dt_s;
    double load = st->true_weight_gn + st->drift_gn;
    st->indicated_weight_gn = sim_approach(st->indicated_weight_gn, load, dt, cfg->scale_response_ms);
}

// Produce the value the firmware would actually read, applying noise and
// the display quantisation. Separated from the physics step so the reading
// rate is independent of the integration rate.
static double sim_plant_sample_reading(const sim_plant_config_t *cfg, sim_plant_state_t *st) {
    double reading = st->indicated_weight_gn;
    if (cfg->scale_noise_gn > 0.0f) {
        reading += sim_rand_normal(st) * cfg->scale_noise_gn;
    }
    if (cfg->scale_resolution_gn > 0.0f) {
        reading = floor(reading / cfg->scale_resolution_gn + 0.5) * cfg->scale_resolution_gn;
    }
    return reading;
}

// Exposed for the scale shim: advances stability tracking and returns the
// newly quantised reading. Call once per scale_period_ms.
double sim_plant_take_reading(const sim_plant_config_t *cfg, sim_plant_state_t *st, uint32_t dt_ms) {
    double previous = st->reported_weight_gn;
    double reading = sim_plant_sample_reading(cfg, st);

    if (fabs(reading - previous) <= (double)cfg->scale_stable_band_gn) {
        st->stable_accum_ms += dt_ms;
    } else {
        st->stable_accum_ms = 0;
    }
    st->stable = st->stable_accum_ms >= cfg->scale_stable_ms;
    st->reported_weight_gn = reading;
    return reading;
}

void sim_plant_format_ad_line(const sim_plant_state_t *st, char *out, int out_len) {
    if (out == NULL || out_len <= 0) {
        return;
    }
    // A&D FX-i streams grams with 4 decimals for a 0.001 g scale, prefixed
    // "ST," when stable and "US," while the reading is still moving.
    double grams = st->reported_weight_gn / GRAINS_PER_GRAM;
    snprintf(out, (size_t)out_len, "%s,%+09.4f g",
             st->stable ? "ST" : "US", grams);
}

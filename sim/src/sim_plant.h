// Physical plant model for the OpenTrickler simulator.
//
// Two coupled models:
//   1. The trickler: two stepper-driven tubes (coarse + fine) whose powder
//      flow lags the commanded speed, and which keep dribbling powder after
//      the motor is told to stop ("tail").
//   2. The scale: an A&D FX-120i, which is what the firmware is normally
//      talking to. See sim_plant.c for the specific behaviours modelled.
#ifndef SIM_PLANT_H_
#define SIM_PLANT_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // ---- Trickler ----
    // Flow is modelled as flow_gps = k * (rps ^ exponent) once spun up.
    float coarse_flow_k;          // grains/sec at 1 rps
    float coarse_flow_exponent;   // >1 = flow grows faster than speed
    float coarse_spinup_ms;       // time constant to reach commanded flow
    float coarse_spindown_ms;     // time constant when commanded to stop
    float coarse_tail_gn;         // powder still in flight/on the tube at stop

    float fine_flow_k;
    float fine_flow_exponent;
    float fine_spinup_ms;
    float fine_spindown_ms;
    float fine_tail_gn;

    // Kernel quantisation: powder leaves in discrete granules. This is the
    // dominant accuracy limit for coarse extruded powders (e.g. N565 at
    // ~0.04gn/kernel) and is why sub-kernel targeting is not achievable.
    float kernel_weight_gn;

    // Multiplicative flow noise (1.0 = +/-100%), models powder bridging and
    // uneven tube fill. Applied per update.
    float flow_noise_frac;

    // ---- A&D FX-120i scale ----
    float scale_resolution_gn;    // display quantum (0.001 g ~= 0.0154 gn)
    float scale_response_ms;      // settling time constant of the reading
    float scale_noise_gn;         // 1-sigma reading noise, in grains
    float scale_drift_gn_per_s;   // slow zero drift
    uint32_t scale_period_ms;     // stream interval between readings
    float scale_stable_band_gn;   // |delta| under this for stable_ms => "ST"
    uint32_t scale_stable_ms;     // dwell required before reporting stable
} sim_plant_config_t;

typedef struct {
    double true_weight_gn;        // actual powder in the pan (whole kernels)
    double pending_gn;            // sub-kernel powder not yet released
    double indicated_weight_gn;   // what the load cell reading is settling to
    double reported_weight_gn;    // last value the firmware was given
    double coarse_flow_gps;       // current actual flow
    double fine_flow_gps;
    double coarse_cmd_rps;        // last commanded speeds
    double fine_cmd_rps;
    double drift_gn;
    bool   stable;
    uint32_t stable_accum_ms;
    uint32_t last_report_tick;
    uint64_t rng_state;
    // Spin-down time constant, latched by sim_plant_step() when each motor's
    // command drops to zero so the tail totals the configured tail mass.
    double coarse_stop_tau_ms;
    double fine_stop_tau_ms;
} sim_plant_state_t;

void sim_plant_defaults(sim_plant_config_t *cfg);
void sim_plant_reset(const sim_plant_config_t *cfg, sim_plant_state_t *st, uint64_t seed);

// Advance the plant by dt_ms of virtual time.
void sim_plant_step(const sim_plant_config_t *cfg, sim_plant_state_t *st, uint32_t dt_ms);

// Called by the motor shim whenever the firmware commands a motor.
void sim_plant_set_coarse_rps(sim_plant_state_t *st, float rps);
void sim_plant_set_fine_rps(sim_plant_state_t *st, float rps);

// Zero / tare, as the firmware's force-zero does.
void sim_plant_zero(sim_plant_state_t *st);

// Empty the pan (cup removed and returned).
void sim_plant_empty_pan(sim_plant_state_t *st);

// Format a reading the way an A&D FX-i streams it, e.g. "ST,+0000.123 g".
// Provided so a serial bridge can reuse the exact wire format.
void sim_plant_format_ad_line(const sim_plant_state_t *st, char *out, int out_len);

#ifdef __cplusplus
}
#endif

#endif  // SIM_PLANT_H_

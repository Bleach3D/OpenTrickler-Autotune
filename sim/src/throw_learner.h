// Learn-from-every-throw threshold tuning for the PID controller (prototype).
//
// The PID charge loop stops the coarse tube when the remaining weight drops
// below coarse_stop_threshold, and stops everything when it drops below
// fine_stop_threshold. Both have to swallow powder that is still in flight
// plus the scale's reporting lag, which change with powder, speed and day.
// Rather than predicting that from a one-off characterization, this measures
// what actually happened on each real charge and nudges the two thresholds:
//
//   fine stop:   the observed fine tail (final - reading at the stop) is
//                exactly what the threshold should have been. Track it,
//                aiming a fraction of the tolerance short. An overthrow
//                jumps straight past it; an under charge only creeps.
//
//   coarse stop: what is left for the fine tube once the coarse tail has
//                landed should be about `fine_reserve_gn`. Too little is an
//                overthrow risk and is corrected at once; too much just
//                wastes time and is trimmed gradually.
//
// Written in plain C with no simulator dependencies so it can move into the
// firmware unchanged if it proves itself.
#ifndef THROW_LEARNER_H_
#define THROW_LEARNER_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // Learned values, fed to the PID loop as its two stop thresholds.
    float coarse_stop_gn;
    float fine_stop_gn;

    // Settings.
    float fine_reserve_gn;      // what coarse should leave for the fine tube
    float accept_tolerance_gn;  // the Accepted Charge Tolerance
    float coarse_stop_min_gn;
    float coarse_stop_max_gn;
    float fine_stop_max_gn;

    // Running statistics of the observed fine tail. It varies from charge to
    // charge (flow noise, kernels, scale quantisation), so the fine stop
    // covers mean + TL_FINE_SPREAD_MULT * typical deviation, not just the
    // mean -- aiming at the mean overthrows on every bigger-than-average tail.
    float fine_tail_mean_gn;
    float fine_tail_dev_gn;
    bool have_fine_stats;

    // Same for how much coarse leaves once its tail has landed: the coarse
    // stop keeps the reserve plus room for a worse-than-average landing.
    float coarse_left_dev_gn;
    float coarse_left_mean_gn;
    bool have_coarse_stats;

    uint16_t charges_learned;
} throw_learner_t;

// What the charge loop records on one charge. Readings are what the
// firmware saw from the scale (not the true powder weight); NAN = missing.
typedef struct {
    float target_gn;
    bool coarse_ran;
    float coarse_stop_reading_gn;   // at the moment coarse was stopped
    float after_coarse_reading_gn;  // ~1.5 s after the coarse stop, NAN if the charge ended first
    float fine_stop_reading_gn;     // at the final motor stop
    float final_reading_gn;         // settled, before the cup is lifted
} throw_observation_t;

typedef struct {
    float fine_tail_gn;             // observed final - fine stop reading
    float remaining_after_coarse_gn;
    bool overthrow;
    bool coarse_backoff;            // coarse stop was raised in a hurry
    bool fine_backoff;              // fine stop was raised in a hurry
} throw_learner_result_t;

void throw_learner_init(throw_learner_t *l, float coarse_stop_gn, float fine_stop_gn,
                        float fine_reserve_gn, float accept_tolerance_gn);

// Fold one charge into the learned thresholds. Returns false (and changes
// nothing) if the observation is unusable.
bool throw_learner_update(throw_learner_t *l, const throw_observation_t *obs,
                          throw_learner_result_t *out);

#ifdef __cplusplus
}
#endif

#endif  // THROW_LEARNER_H_

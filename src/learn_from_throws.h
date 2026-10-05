#ifndef LEARN_FROM_THROWS_H_
#define LEARN_FROM_THROWS_H_

// Learn from throws: an optional, per-profile layer on the PID controller.
// While enabled for the selected profile, PID charges use that profile's
// learned coarse and fine stop thresholds instead of the global ones, and
// every completed PID charge updates them (see throw_learner.h for how).
//
// This is independent of AI Tuning: it never runs a characterization, does
// not touch the AI model, and has no effect on the adaptive controller or on
// AI characterization samples.

#include <stdbool.h>
#include <stdint.h>
#include "throw_learner.h"
#include "http_rest.h"

#define LEARN_FROM_THROWS_REV               1
#define LEARN_FROM_THROWS_DEFAULT_RESERVE   0.60f

typedef struct {
    bool enabled;
    throw_learner_t learner;
} learn_from_throws_profile_t;

// Last charge folded in, for the web GUI. RAM only.
typedef struct {
    bool valid;
    uint8_t profile_idx;
    float error_gn;                 // settled reading - target, before any top-up
    float coarse_stop_used_gn;
    float fine_stop_used_gn;
    throw_learner_result_t result;
} learn_from_throws_last_t;

#ifdef __cplusplus
extern "C" {
#endif

bool learn_from_throws_init(void);
bool learn_from_throws_save(void);

// Charge-mode hooks. thresholds() returns true, and fills the two values, if
// learning is enabled for profile_idx; otherwise it leaves them untouched.
bool learn_from_throws_thresholds(uint8_t profile_idx, float *coarse_stop_gn, float *fine_stop_gn);
void learn_from_throws_record(uint8_t profile_idx, const throw_observation_t *obs,
                              float accept_tolerance_gn,
                              float coarse_stop_used_gn, float fine_stop_used_gn);

// Settings / inspection (also used by the simulator).
bool learn_from_throws_get(uint8_t profile_idx, learn_from_throws_profile_t *out);
bool learn_from_throws_get_last(learn_from_throws_last_t *out);
bool learn_from_throws_set_enabled(uint8_t profile_idx, bool enabled);
bool learn_from_throws_set_reserve(uint8_t profile_idx, float reserve_gn);
bool learn_from_throws_reset(uint8_t profile_idx);

bool http_rest_learn_from_throws(struct fs_file *file, int num_params, char *params[], char *values[]);

#ifdef __cplusplus
}
#endif

#endif  // LEARN_FROM_THROWS_H_

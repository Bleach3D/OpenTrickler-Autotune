#ifndef PID_AUTOTUNE_H_
#define PID_AUTOTUNE_H_

#include <stdint.h>
#include <stdbool.h>
#include "http_rest.h"

// PID Tuning ("autotune"): characterises a powder's flow rate and the
// scale's reporting lag on both tubes by running a ladder of fixed-speed,
// fixed-time throws, then fits coarse/fine speed limits, Kp and the
// coarse/fine handoff (stop) threshold from that data -- the same numbers
// you would otherwise hand-tune on the Profile settings page, just derived
// from measurement instead of guesswork.
//
// This is a separate, from-scratch port of the "Learn Powder" approach from
// https://github.com/magnaludus/OpenTrickler-RP2040-Controller, adapted to
// this fork's own PID-only charge_mode (no lag-compensated stop prediction
// in this firmware, so the handoff always has to swallow the whole
// measured tail, not just its scatter) and driven from the web GUI over
// REST rather than the physical menu.
//
// v1 scope, deliberately: characterise + fit only. No automated
// confirm-throws-and-back-off loop, and no automatic cup-dump handling --
// if the cup would overflow mid-run the run aborts with a clear message
// asking you to empty it and start again, rather than pausing to wait for
// that. Fitted numbers are applied to the selected profile in RAM only;
// you test them with ordinary charges and use the existing Profile page
// Apply / Save to EEPROM to keep them, exactly like reviewing an AI Tuning
// suggestion today.

#define PID_AUTOTUNE_THROWS_PER_PHASE      12
#define PID_AUTOTUNE_COARSE_SPEED_LEVELS   4       // 12 throws = 4 speeds x 3
#define PID_AUTOTUNE_FINE_SPEED_LEVELS     3       // 12 throws = 3 speeds x 4

typedef enum {
    PID_AUTOTUNE_STATE_IDLE = 0,
    PID_AUTOTUNE_STATE_WAIT_FOR_ZERO = 1,
    PID_AUTOTUNE_STATE_COARSE = 2,
    PID_AUTOTUNE_STATE_FINE = 3,
    PID_AUTOTUNE_STATE_FIT = 4,
    PID_AUTOTUNE_STATE_DONE = 5,
    PID_AUTOTUNE_STATE_ABORTED = 6,
    PID_AUTOTUNE_STATE_ERROR = 7,
} pid_autotune_state_t;

typedef struct {
    float speed_rps;
    float time_s;           // motor start to stop
    float stop_weight;      // reading when the motor was told to stop
    float settled_weight;   // reading after the scale settled
    float flow_gps;         // settled_weight / time_s
    float tail;             // settled_weight - stop_weight
    float lag_s;            // tail / flow_gps, the scale lag measured on this throw
} pid_autotune_throw_t;

typedef struct {
    // Fitted flow/lag
    float coarse_k;                 // gr/s per rps
    float fine_k;                   // gr/s per rps
    float coarse_lag_s;
    float fine_lag_s;
    float coarse_lag_sd_s;
    float fine_lag_sd_s;

    // Profile proposal (same fields as profile_t / charge_mode's own coarse_stop_threshold)
    float coarse_min_rps;
    float coarse_max_rps;
    float coarse_kp;
    float fine_min_rps;
    float fine_max_rps;
    float fine_kp;
    float fine_taper_gr;
    float coarse_stop_threshold;

    // Predicted throw at the confirm target with the fitted profile, for
    // reference only -- not measured, and not confirmed with real throws
    // in this version.
    float predicted_coarse_s;
    float predicted_fine_s;
    float predicted_total_s;
    bool meets_time_goal;
} pid_autotune_result_t;

#define EEPROM_PID_AUTOTUNE_CONFIG_REV     1

typedef struct {
    uint16_t pid_autotune_config_rev;

    float coarse_target_gr;         // grams landed per coarse throw at the ladder's top speed, default 8.0
    float fine_target_gr;           // default 1.75
    float coarse_speed_ceiling;     // highest coarse speed the ladder will use (rps)
    float fine_speed_ceiling;       // highest fine speed the ladder will use (rps)
    float confirm_target_gr;        // charge weight the fit optimises for (your usual charge weight)
    float time_goal_s;              // upper limit on throw time at that weight, default 7.0
    float cup_capacity_gr;          // abort rather than overflow the cup, default 250

    // Multiple of the coarse tube's own measured 3-sigma stop scatter carried as cushion in the
    // coarse/fine handoff. 1.0 = bare 3 sigma, higher = more conservative, i.e. a bigger, slower
    // but safer handoff. This is the dial to raise if a fitted profile still overthrows.
    float coarse_stop_safety;

    // How many standard deviations of fine landing error have to fit inside the Accepted Charge
    // Tolerance, which is what caps the fine landing speed. Higher = slower, more accurate landing.
    float land_sigma;
} pid_autotune_config_t;

typedef struct {
    pid_autotune_config_t config;
    pid_autotune_state_t state;
    uint8_t throw_idx;              // within the current phase
    float current_speed;
    pid_autotune_throw_t coarse[PID_AUTOTUNE_THROWS_PER_PHASE];
    pid_autotune_throw_t fine[PID_AUTOTUNE_THROWS_PER_PHASE];
    pid_autotune_result_t result;
    float cup_load_gr;              // powder dispensed into the cup this run
    bool result_valid;
    bool applied_to_profile;
    char message[48];
} pid_autotune_t;


#ifdef __cplusplus
extern "C" {
#endif

bool pid_autotune_init(void);
bool pid_autotune_config_save(void);

// Runs the whole characterise + fit routine. Entered from menu.c the same
// way charge_mode_menu()/learn-style modes are, via APP_STATE_ENTER_PID_AUTOTUNE_FROM_REST.
// Returns the mui form id to go back to.
uint8_t pid_autotune_menu(void);

bool pid_autotune_apply_to_profile(void);   // write the fitted values into the selected profile (RAM)
void pid_autotune_request_abort(void);      // signal the running characterisation to stop
bool pid_autotune_is_active(void);          // true while pid_autotune_menu() owns the device (run or result screen)
void pid_autotune_request_restart(void);    // run again when the result/error screen is next woken

// REST
bool http_rest_pid_autotune_state(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_pid_autotune_config(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_pid_autotune_config_set(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_pid_autotune_start(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_pid_autotune_action(struct fs_file *file, int num_params, char *params[], char *values[]);

extern pid_autotune_t pid_autotune;

#ifdef __cplusplus
}
#endif

#endif  // PID_AUTOTUNE_H_

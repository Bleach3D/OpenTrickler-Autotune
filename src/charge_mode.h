#ifndef CHARGE_MODE_H_
#define CHARGE_MODE_H_

#include <stdint.h>
#include "http_rest.h"
#include "common.h"
#include "neopixel_led.h"
#include "profile.h"


#define EEPROM_CHARGE_MODE_DATA_REV                     19             // Added reverse tube (coarse/fine anti-dribble)

#define WEIGHT_STRING_LEN 8

typedef enum {
    CHARGE_MODE_EXIT = 0,
    CHARGE_MODE_WAIT_FOR_ZERO = 1,
    CHARGE_MODE_WAIT_FOR_COMPLETE = 2,
    CHARGE_MODE_WAIT_FOR_CUP_REMOVAL = 3,
    CHARGE_MODE_WAIT_FOR_CUP_RETURN = 4,
    CHARGE_MODE_STABILIZING = 5,
    // Automatic dispensing has stopped because the remaining amount is less
    // than one powder kernel and adding a whole kernel would overshoot the
    // Accepted Charge Tolerance. Waiting for the operator to add hand-cut
    // partial kernels. See charge_mode_manual_finish() in charge_mode.cpp.
    CHARGE_MODE_MANUAL_FINISH = 6,
} charge_mode_state_t;

// NOTE: One of these exists per profile (see eeprom_charge_mode_profile_data_t
// below). "charge_mode_config.eeprom_charge_mode_data" always holds a working
// copy of whichever profile is currently active; charge_mode_data_load_for_profile()
// refreshes that copy whenever the active profile changes.
typedef struct {
    uint16_t charge_mode_data_rev;

    float coarse_stop_threshold;
    float fine_stop_threshold;

    float set_point_sd_margin;
    float set_point_mean_margin;
    float coarse_stop_gate_ratio; // 0.0=open, 1.0=close, -1.0=disabled (optional)

    decimal_places_t decimal_places;

    // Precharge
    bool precharge_enable;
    uint32_t precharge_time_ms;
    float precharge_speed_rps;

    // AI tuning time targets
    uint32_t coarse_time_target_ms;
    uint32_t total_time_target_ms;

    // ML data collection during normal (non-tuning) charges
    bool ml_data_collection_enabled;

    // Auto zero scale when cup is returned
    bool auto_zero_on_cup_return;

    // Pulse mode - helps with slow scales near target
    bool pulse_mode_enabled;
    float pulse_threshold;          // Start pulsing when error < this (grains)
    uint32_t pulse_duration_ms;     // Motor on time per pulse
    uint32_t pulse_wait_ms;         // Wait time between pulses for scale to update

    // Scale stabilization after motors stop (before overthrow/underthrow decision)
    bool stabilization_enabled;     // true = fixed wait, false = adaptive SD-based
    uint32_t stabilization_time_ms; // Fixed wait time when enabled (default 2000ms)

    // When false (default), the adaptive/AI controller may never hand off from
    // the coarse trickler earlier than coarse_stop_threshold asks for: the
    // configured value is a hard ceiling on the handoff margin, subject only to
    // the physical tail-clearance floor needed to stop the coarse motor without
    // it overshooting on its own momentum.
    // When true, the controller uses its own learned margin (original
    // behaviour) -- safer against coarse overshoot, but it can leave several
    // grains for the fine tube on larger charges, which is slow and less
    // repeatable because the fine tube ends up working far outside the small
    // tail regime it was characterized in.
    bool ai_coarse_stop_auto;

    // 0.0 = prioritise speed, 1.0 = prioritise accuracy, 0.5 = balanced.
    // Scales how much finishing margin and settle patience the fine phase gets.
    float ai_accuracy_speed_bias;

    // How close to target a finished charge must be to count as good, in
    // grains. This is the +/- band that decides the green "accepted" result
    // versus an under/over warning, and it is also the band the controller
    // works to: it will keep topping up while the charge is more than this
    // under target rather than handing back a short charge.
    // Previously this was a hardcoded 0.0205 gn with no way to change it.
    float accept_tolerance_gn;

    // Controller selection.
    //
    // false (default): the classic PID controller is the only path. Every
    //   number that decides the charge -- the PID gains, the speed limits and
    //   the stop thresholds -- lives in the profile where you can see and
    //   edit it. AI characterization becomes an auto-tune that *suggests*
    //   values for those fields rather than a second controller.
    //
    // true: the legacy adaptive controller, which computes its own margins at
    //   runtime and overrides the profile settings. Kept so existing setups
    //   can fall back, but it is not the recommended path: its stop margins
    //   are recalculated in several layers and are not user controllable.
    bool use_adaptive_controller;

    // Manual finish, for powders whose kernel is too large to reliably
    // finish automatically within the Accepted Charge Tolerance (e.g.
    // N565/N570-class extruded powders at ~0.08gn/kernel with a 0.02gn or
    // tighter tolerance). When the remaining amount drops below one kernel
    // and dispensing a whole kernel would overshoot tolerance, the
    // controller stops and asks the operator to add hand-cut partial
    // kernels instead of guessing at continuous flow it cannot deliver.
    bool manual_finish_enabled;
    // Weight of a hand-cut kernel fragment, in grains, e.g. 0.04 for a
    // kernel cut in half or 0.02 for quarters. 0 = not set, in which case
    // manual finish falls back to whole-kernel-sized suggestions, which
    // is only useful for confirming the remaining amount rather than for
    // closing it.
    float manual_finish_cut_kernel_gn;

    // Reverse tube: after a tube throws its own final stop for this charge
    // (not on abort), briefly reverse that tube's motor to pull back the
    // last bit of powder still sitting in the tube/gate, reducing post-stop
    // dribble. Configured independently per tube because the coarse and
    // fine tubes carry very different amounts of residual powder and are
    // driven by different motors. Reuses that tube's own configured minimum
    // flow speed rather than adding a separate reverse-speed field.
    bool coarse_reverse_enabled;
    float coarse_reverse_revolutions;
    bool fine_reverse_enabled;
    float fine_reverse_revolutions;

    // LED related settings
    rgbw_u32_t neopixel_normal_charge_colour;
    rgbw_u32_t neopixel_under_charge_colour;
    rgbw_u32_t neopixel_over_charge_colour;
    rgbw_u32_t neopixel_not_ready_colour;

} eeprom_charge_mode_data_t;

// Persisted storage: one eeprom_charge_mode_data_t per profile slot, indexed
// the same way as eeprom_profile_data_t.profiles[] in profile.h.
typedef struct {
    uint16_t charge_mode_data_rev;
    eeprom_charge_mode_data_t profiles[MAX_PROFILE_CNT];
} eeprom_charge_mode_profile_data_t;

typedef struct {
    eeprom_charge_mode_data_t eeprom_charge_mode_data;
    float target_charge_weight;
    uint32_t charge_mode_event;
    charge_mode_state_t charge_mode_state;

    // Which profile slot eeprom_charge_mode_data was last loaded from.
    uint8_t loaded_profile_idx;
} charge_mode_config_t;


// C Functions
#ifdef __cplusplus
extern "C" {
#endif


bool charge_mode_config_init(void);
uint8_t charge_mode_menu(bool charge_mode_skip_user_input);
float charge_mode_target_weight_from_digits(void);
void charge_mode_get_live_motor_commands(float *coarse_rps, float *fine_rps);
bool charge_mode_config_save(void);
bool charge_mode_is_menu_active(void);

// Per-profile charge mode config
// Copies profile_idx's stored charge mode settings into the active working
// copy (charge_mode_config.eeprom_charge_mode_data). Call this any time the
// selected profile changes so the correct charge mode settings are used.
bool charge_mode_data_load_for_profile(uint8_t profile_idx);
uint8_t charge_mode_data_get_loaded_profile_idx(void);

// Live manual-finish status, for the on-device render task and REST state
// endpoint. Only meaningful while charge_mode_config.charge_mode_state ==
// CHARGE_MODE_MANUAL_FINISH; suggested_kernels/remaining/cut_kernel_gn are
// undefined otherwise.
bool charge_mode_is_manual_finish_active(void);
float charge_mode_get_manual_finish_remaining_gn(void);
int charge_mode_get_manual_finish_suggested_kernels(void);
float charge_mode_get_manual_finish_cut_kernel_gn(void);

// REST interface
bool http_rest_charge_mode_config(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_charge_mode_state(struct fs_file *file, int num_params, char *params[], char *values[]);


#ifdef __cplusplus
}  // __cplusplus
#endif


#endif  // CHARGE_MODE_H_

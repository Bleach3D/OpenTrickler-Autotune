#ifndef CHARGE_TIMELINE_H_
#define CHARGE_TIMELINE_H_

#include <stdint.h>
#include <stdbool.h>
#include "http_rest.h"

// Records the weight / motor-speed timeline of one charge for the web UI's
// "Charge Timeline" card. A low-priority task samples every 100 ms while a
// charge is being recorded; when the buffer fills up it is decimated by 2 and
// the sample interval doubles, so a charge of any length fits.

#define CHARGE_TIMELINE_MAX_SAMPLES     600
#define CHARGE_TIMELINE_BASE_PERIOD_MS  100

typedef enum {
    CHARGE_TIMELINE_PHASE_IDLE = 0,     // charging started, no motor running yet
    CHARGE_TIMELINE_PHASE_COARSE = 1,
    CHARGE_TIMELINE_PHASE_FINE = 2,
    CHARGE_TIMELINE_PHASE_SETTLE = 3,   // motors stopped after having run
} charge_timeline_phase_t;

#ifdef __cplusplus
extern "C" {
#endif

void charge_timeline_init(void);

// Start a new recording (a previous one is replaced).
void charge_timeline_begin(float target_weight, float tolerance);

// Finish the current recording with the settled final weight.
void charge_timeline_finish(bool final_valid, float final_weight);

// Stop the current recording without a result (charge aborted).
void charge_timeline_abort(void);

bool http_rest_charge_timeline(struct fs_file *file, int num_params, char *params[], char *values[]);

#ifdef __cplusplus
}
#endif

#endif  // CHARGE_TIMELINE_H_

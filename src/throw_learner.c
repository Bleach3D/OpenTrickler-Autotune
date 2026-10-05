#include <math.h>
#include <string.h>

#include "throw_learner.h"

// How far short of target the fine stop aims, as a fraction of the
// tolerance. Landing a little under is recoverable; over is not.
#define TL_FINE_AIM_UNDER_FRAC      0.25f
// Smoothing for ordinary (non-overthrow) charges: the share of the gap
// between the current and the ideal threshold closed per charge.
#define TL_FINE_GAIN                0.30f
#define TL_COARSE_GAIN              0.50f
// Fine stop covers the mean tail plus this many typical (mean absolute)
// deviations, which is roughly the 95th percentile of the tails seen.
#define TL_FINE_SPREAD_MULT         2.0f
// The coarse stop keeps reserve + this many typical deviations of how much
// coarse leaves.
#define TL_COARSE_SPREAD_MULT       2.0f
// Smoothing of the tail statistics.
#define TL_STATS_GAIN               0.25f
// After an overthrow the fine stop goes straight to the ideal value plus
// this fraction of the tolerance.
#define TL_FINE_OVER_EXTRA_FRAC     0.50f
// Coarse is "too close" when what it left is below this share of the
// reserve; it is then raised by the full shortfall plus this share again.
#define TL_COARSE_DANGER_FRAC       0.50f
#define TL_COARSE_OVER_EXTRA_FRAC   0.25f
// A fine tail above this is not believed (cup knocked, scale glitch). This
// never discards an overthrow caused by coarse: those are learned below.
#define TL_MAX_PLAUSIBLE_FINE_TAIL  1.50f

static float tl_clampf(float v, float lo, float hi) {
    if (!(v >= lo)) return lo;
    if (v > hi) return hi;
    return v;
}

void throw_learner_init(throw_learner_t *l, float coarse_stop_gn, float fine_stop_gn,
                        float fine_reserve_gn, float accept_tolerance_gn) {
    memset(l, 0, sizeof(*l));
    l->fine_reserve_gn = fine_reserve_gn;
    l->accept_tolerance_gn = accept_tolerance_gn;
    l->coarse_stop_min_gn = 0.20f;
    l->coarse_stop_max_gn = 10.0f;
    l->fine_stop_max_gn = 0.60f;
    l->coarse_stop_gn = tl_clampf(coarse_stop_gn, l->coarse_stop_min_gn, l->coarse_stop_max_gn);
    l->fine_stop_gn = tl_clampf(fine_stop_gn, 0.0f, l->fine_stop_max_gn);
}

bool throw_learner_update(throw_learner_t *l, const throw_observation_t *obs,
                          throw_learner_result_t *out) {
    throw_learner_result_t r;
    memset(&r, 0, sizeof(r));
    r.fine_tail_gn = NAN;
    r.remaining_after_coarse_gn = NAN;

    if (!isfinite(obs->target_gn) || !isfinite(obs->final_reading_gn) ||
        !isfinite(obs->fine_stop_reading_gn)) {
        return false;
    }

    const float tol = fmaxf(l->accept_tolerance_gn, 0.005f);
    const float error = obs->final_reading_gn - obs->target_gn;
    r.overthrow = error > tol;

    // ---- Fine stop -------------------------------------------------------
    // The PID loop stopped when (target - reading) <= fine_stop, and the
    // powder that landed afterwards was fine_tail. The threshold that would
    // have landed exactly on target is therefore fine_tail itself.
    //
    // Only learn this when the fine tube actually had the last word: if the
    // charge ended within the after-coarse window, what landed after the
    // stop was mostly coarse powder still in flight, which is the coarse
    // stop's problem (handled below), not a fine tail.
    bool fine_had_last_word = !obs->coarse_ran || isfinite(obs->after_coarse_reading_gn);
    float fine_tail = obs->final_reading_gn - obs->fine_stop_reading_gn;
    if (fine_had_last_word && fine_tail >= -tol && fine_tail <= TL_MAX_PLAUSIBLE_FINE_TAIL) {
        fine_tail = fmaxf(fine_tail, 0.0f);
        r.fine_tail_gn = fine_tail;

        if (!l->have_fine_stats) {
            l->fine_tail_mean_gn = fine_tail;
            l->fine_tail_dev_gn = tol * 0.5f;
            l->have_fine_stats = true;
        } else {
            l->fine_tail_dev_gn += TL_STATS_GAIN * (fabsf(fine_tail - l->fine_tail_mean_gn) - l->fine_tail_dev_gn);
            l->fine_tail_mean_gn += TL_STATS_GAIN * (fine_tail - l->fine_tail_mean_gn);
        }

        const float fine_ideal = l->fine_tail_mean_gn +
                                 TL_FINE_SPREAD_MULT * l->fine_tail_dev_gn +
                                 tol * TL_FINE_AIM_UNDER_FRAC;
        float fine_stop = l->fine_stop_gn;
        if (r.overthrow) {
            // This charge's own tail, not just the statistics, has to be covered.
            fine_stop = fmaxf(fmaxf(fine_stop, fine_ideal), fine_tail + tol * TL_FINE_AIM_UNDER_FRAC) +
                        tol * TL_FINE_OVER_EXTRA_FRAC;
            r.fine_backoff = true;
        } else {
            fine_stop += TL_FINE_GAIN * (fine_ideal - fine_stop);
        }
        l->fine_stop_gn = tl_clampf(fine_stop, 0.0f, l->fine_stop_max_gn);
    }
    else if (!obs->coarse_ran) {
        // No coarse to blame and an implausible tail: a glitch, ignore it.
        return false;
    }

    // ---- Coarse stop -----------------------------------------------------
    if (obs->coarse_ran) {
        // If the charge finished before the after-coarse reading was taken,
        // coarse left almost nothing: use the final reading, which reads as
        // "nothing left" and pushes the coarse stop up.
        float after = isfinite(obs->after_coarse_reading_gn)
                          ? obs->after_coarse_reading_gn
                          : obs->final_reading_gn;
        float remaining = obs->target_gn - after;
        r.remaining_after_coarse_gn = remaining;

        // Spread is measured in how much coarse left. Its mean moves with
        // the coarse stop itself, so the deviation is taken around a running
        // mean of the landing *error* (left - what this stop aimed to leave),
        // which stays put when the stop changes.
        // Only charges that lasted past the after-coarse reading measure a
        // real landing; one that ended sooner overshot so far that it would
        // swamp the statistics (it still triggers the back-off below).
        float landing_error = remaining - l->coarse_stop_gn;
        if (!isfinite(obs->after_coarse_reading_gn)) {
            // no statistics update
        } else if (!l->have_coarse_stats) {
            l->coarse_left_mean_gn = landing_error;
            l->coarse_left_dev_gn = 0.0f;
            l->have_coarse_stats = true;
        } else {
            l->coarse_left_dev_gn += TL_STATS_GAIN * (fabsf(landing_error - l->coarse_left_mean_gn) - l->coarse_left_dev_gn);
            l->coarse_left_mean_gn += TL_STATS_GAIN * (landing_error - l->coarse_left_mean_gn);
        }

        const float reserve = fmaxf(l->fine_reserve_gn, l->fine_stop_gn + tol) +
                              TL_COARSE_SPREAD_MULT * l->coarse_left_dev_gn;
        float shortfall = reserve - remaining;   // > 0: coarse got too close
        float coarse_stop = l->coarse_stop_gn;
        bool danger = remaining < reserve * TL_COARSE_DANGER_FRAC ||
                      (r.overthrow && remaining < reserve);
        if (danger) {
            coarse_stop += fmaxf(shortfall, 0.0f) + reserve * TL_COARSE_OVER_EXTRA_FRAC;
            r.coarse_backoff = true;
        } else {
            coarse_stop += TL_COARSE_GAIN * shortfall;
        }
        float coarse_floor = fmaxf(l->coarse_stop_min_gn, l->fine_stop_gn * 2.0f);
        l->coarse_stop_gn = tl_clampf(coarse_stop, coarse_floor, l->coarse_stop_max_gn);
    }

    if (l->charges_learned < UINT16_MAX) {
        l->charges_learned++;
    }
    if (out != NULL) {
        *out = r;
    }
    return true;
}

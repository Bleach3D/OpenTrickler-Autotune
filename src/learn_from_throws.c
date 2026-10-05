#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "learn_from_throws.h"
#include "charge_mode.h"
#include "common.h"
#include "eeprom.h"
#include "profile.h"

typedef struct {
    uint16_t rev;
    learn_from_throws_profile_t profiles[MAX_PROFILE_CNT];
} eeprom_learn_from_throws_t;

static eeprom_learn_from_throws_t g_ltl;
static learn_from_throws_last_t g_last;

// Start (or restart) a profile's learning from the thresholds that profile
// is set to use today, so switching it on never changes the first charge --
// the learning only moves from there.
static void ltl_seed(uint8_t profile_idx, float reserve_gn) {
    float coarse_stop = 1.0f, fine_stop = 0.03f, tolerance = 0.0205f;
    charge_mode_get_profile_thresholds(profile_idx, &coarse_stop, &fine_stop, &tolerance);
    if (!(isfinite(tolerance) && tolerance > 0.0f)) {
        tolerance = 0.0205f;
    }
    throw_learner_init(&g_ltl.profiles[profile_idx].learner,
                       coarse_stop, fine_stop, reserve_gn, tolerance);
}

static float ltl_valid_reserve(float reserve_gn) {
    if (!isfinite(reserve_gn) || reserve_gn <= 0.0f) {
        return LEARN_FROM_THROWS_DEFAULT_RESERVE;
    }
    return fminf(fmaxf(reserve_gn, 0.20f), 3.00f);
}

bool learn_from_throws_save(void) {
    g_ltl.rev = LEARN_FROM_THROWS_REV;
    return eeprom_write(EEPROM_LEARN_FROM_THROWS_BASE_ADDR, (uint8_t *)&g_ltl, sizeof(g_ltl));
}

bool learn_from_throws_init(void) {
    memset(&g_ltl, 0, sizeof(g_ltl));
    memset(&g_last, 0, sizeof(g_last));

    bool ok = eeprom_read(EEPROM_LEARN_FROM_THROWS_BASE_ADDR, (uint8_t *)&g_ltl, sizeof(g_ltl));
    if (!ok || g_ltl.rev != LEARN_FROM_THROWS_REV) {
        // Nothing stored yet (or an older layout): everything off. Learners
        // are seeded when a profile is first switched on.
        memset(&g_ltl, 0, sizeof(g_ltl));
        g_ltl.rev = LEARN_FROM_THROWS_REV;
    }

    for (int i = 0; i < MAX_PROFILE_CNT; i++) {
        learn_from_throws_profile_t *p = &g_ltl.profiles[i];
        throw_learner_t *l = &p->learner;
        // Never trust stored numbers blindly: a corrupt slot is switched off
        // and reseeded rather than steering a charge.
        bool sane = isfinite(l->coarse_stop_gn) && isfinite(l->fine_stop_gn) &&
                    isfinite(l->fine_reserve_gn) && l->fine_reserve_gn > 0.0f &&
                    l->coarse_stop_gn >= 0.0f && l->coarse_stop_gn <= 20.0f &&
                    l->fine_stop_gn >= 0.0f && l->fine_stop_gn <= 2.0f;
        if (!sane) {
            p->enabled = false;
            ltl_seed((uint8_t)i, LEARN_FROM_THROWS_DEFAULT_RESERVE);
        }
    }

    eeprom_register_handler(learn_from_throws_save);
    return true;
}

bool learn_from_throws_thresholds(uint8_t profile_idx, float *coarse_stop_gn, float *fine_stop_gn) {
    if (profile_idx >= MAX_PROFILE_CNT || !g_ltl.profiles[profile_idx].enabled) {
        return false;
    }
    const throw_learner_t *l = &g_ltl.profiles[profile_idx].learner;
    if (coarse_stop_gn != NULL) *coarse_stop_gn = l->coarse_stop_gn;
    if (fine_stop_gn != NULL) *fine_stop_gn = l->fine_stop_gn;
    return true;
}

void learn_from_throws_record(uint8_t profile_idx, const throw_observation_t *obs,
                              float accept_tolerance_gn,
                              float coarse_stop_used_gn, float fine_stop_used_gn) {
    if (profile_idx >= MAX_PROFILE_CNT || obs == NULL || !g_ltl.profiles[profile_idx].enabled) {
        return;
    }
    throw_learner_t *l = &g_ltl.profiles[profile_idx].learner;
    if (isfinite(accept_tolerance_gn) && accept_tolerance_gn > 0.0f) {
        l->accept_tolerance_gn = accept_tolerance_gn;
    }

    throw_learner_result_t result;
    if (!throw_learner_update(l, obs, &result)) {
        return;
    }

    g_last.valid = true;
    g_last.profile_idx = profile_idx;
    g_last.error_gn = obs->final_reading_gn - obs->target_gn;
    g_last.coarse_stop_used_gn = coarse_stop_used_gn;
    g_last.fine_stop_used_gn = fine_stop_used_gn;
    g_last.result = result;

    // Saved straight away so a power cycle never loses what the last charge
    // taught it (a few hundred bytes; far inside the EEPROM's endurance at
    // one write per charge).
    learn_from_throws_save();
}

bool learn_from_throws_get(uint8_t profile_idx, learn_from_throws_profile_t *out) {
    if (profile_idx >= MAX_PROFILE_CNT || out == NULL) {
        return false;
    }
    *out = g_ltl.profiles[profile_idx];
    return true;
}

bool learn_from_throws_get_last(learn_from_throws_last_t *out) {
    if (out == NULL) {
        return false;
    }
    *out = g_last;
    return g_last.valid;
}

bool learn_from_throws_set_enabled(uint8_t profile_idx, bool enabled) {
    if (profile_idx >= MAX_PROFILE_CNT) {
        return false;
    }
    learn_from_throws_profile_t *p = &g_ltl.profiles[profile_idx];
    if (enabled && !p->enabled && p->learner.charges_learned == 0) {
        // First switch-on (or after a reset): start from today's thresholds.
        ltl_seed(profile_idx, ltl_valid_reserve(p->learner.fine_reserve_gn));
    }
    p->enabled = enabled;
    return learn_from_throws_save();
}

bool learn_from_throws_set_reserve(uint8_t profile_idx, float reserve_gn) {
    if (profile_idx >= MAX_PROFILE_CNT) {
        return false;
    }
    g_ltl.profiles[profile_idx].learner.fine_reserve_gn = ltl_valid_reserve(reserve_gn);
    return learn_from_throws_save();
}

bool learn_from_throws_reset(uint8_t profile_idx) {
    if (profile_idx >= MAX_PROFILE_CNT) {
        return false;
    }
    ltl_seed(profile_idx, ltl_valid_reserve(g_ltl.profiles[profile_idx].learner.fine_reserve_gn));
    if (g_last.profile_idx == profile_idx) {
        g_last.valid = false;
    }
    return learn_from_throws_save();
}


// ---------------------------------------------------------------------------
// REST: /rest/learn_from_throws
//   pf (int)        profile index (default: selected profile)
//   enabled (bool)  switch learning on/off for that profile
//   reserve (float) gn coarse should leave for the fine tube
//   reset (any)     forget what was learned and restart from the profile's
//                   own coarse/fine stop thresholds
// Always replies with the profile's state.
// ---------------------------------------------------------------------------
static char ltl_json_buffer[1024];

// JSON has no NaN; unknown values are sent as null.
static const char *ltl_num(char *buf, size_t len, float v, int decimals) {
    if (!isfinite(v)) {
        snprintf(buf, len, "null");
    } else {
        snprintf(buf, len, "%.*f", decimals, v);
    }
    return buf;
}

bool http_rest_learn_from_throws(struct fs_file *file, int num_params, char *params[], char *values[]) {
    int profile_idx = (int)profile_get_selected_idx();
    for (int i = 0; i < num_params; i++) {
        if (strcmp(params[i], "pf") == 0) {
            profile_idx = atoi(values[i]);
        }
    }
    if (profile_idx < 0 || profile_idx >= MAX_PROFILE_CNT) {
        profile_idx = (int)profile_get_selected_idx();
    }

    for (int i = 0; i < num_params; i++) {
        if (strcmp(params[i], "reset") == 0) {
            learn_from_throws_reset((uint8_t)profile_idx);
        }
    }
    for (int i = 0; i < num_params; i++) {
        if (strcmp(params[i], "reserve") == 0) {
            learn_from_throws_set_reserve((uint8_t)profile_idx, strtof(values[i], NULL));
        }
        else if (strcmp(params[i], "enabled") == 0) {
            learn_from_throws_set_enabled((uint8_t)profile_idx, string_to_boolean(values[i]));
        }
    }

    const learn_from_throws_profile_t *p = &g_ltl.profiles[profile_idx];
    const throw_learner_t *l = &p->learner;
    bool have_last = g_last.valid && g_last.profile_idx == profile_idx;
    char a[16], b[16], c[16], d[16], e[16], f[16], g[16];
    float profile_coarse_stop = NAN, profile_fine_stop = NAN;
    charge_mode_get_profile_thresholds((uint8_t)profile_idx, &profile_coarse_stop, &profile_fine_stop, NULL);

    int len = snprintf(ltl_json_buffer, sizeof(ltl_json_buffer),
        "%s{\"pf\":%d,\"enabled\":%s,\"coarse_stop\":%.3f,\"fine_stop\":%.3f,"
        "\"reserve\":%.3f,\"charges\":%u,"
        "\"fine_tail_mean\":%s,\"fine_tail_dev\":%s,\"coarse_left_dev\":%s,"
        "\"profile_coarse_stop\":%.3f,\"profile_fine_stop\":%.3f,"
        "\"last\":%s",
        http_json_header,
        profile_idx,
        boolean_to_string(p->enabled),
        l->coarse_stop_gn,
        l->fine_stop_gn,
        l->fine_reserve_gn,
        (unsigned)l->charges_learned,
        l->have_fine_stats ? ltl_num(a, sizeof(a), l->fine_tail_mean_gn, 3) : "null",
        l->have_fine_stats ? ltl_num(b, sizeof(b), l->fine_tail_dev_gn, 3) : "null",
        l->have_coarse_stats ? ltl_num(c, sizeof(c), l->coarse_left_dev_gn, 3) : "null",
        profile_coarse_stop,
        profile_fine_stop,
        have_last ? "{" : "null");
    if (len < 0 || len >= (int)sizeof(ltl_json_buffer)) {
        return false;
    }
    if (have_last) {
        int more = snprintf(ltl_json_buffer + len, sizeof(ltl_json_buffer) - (size_t)len,
            "\"error\":%s,\"coarse_stop_used\":%s,\"fine_stop_used\":%s,"
            "\"fine_tail\":%s,\"coarse_left\":%s,\"overthrow\":%s,"
            "\"coarse_backoff\":%s,\"fine_backoff\":%s}",
            ltl_num(d, sizeof(d), g_last.error_gn, 3),
            ltl_num(e, sizeof(e), g_last.coarse_stop_used_gn, 3),
            ltl_num(f, sizeof(f), g_last.fine_stop_used_gn, 3),
            ltl_num(g, sizeof(g), g_last.result.fine_tail_gn, 3),
            ltl_num(a, sizeof(a), g_last.result.remaining_after_coarse_gn, 3),
            boolean_to_string(g_last.result.overthrow),
            boolean_to_string(g_last.result.coarse_backoff),
            boolean_to_string(g_last.result.fine_backoff));
        if (more < 0 || len + more >= (int)sizeof(ltl_json_buffer)) {
            return false;
        }
        len += more;
    }
    int tail = snprintf(ltl_json_buffer + len, sizeof(ltl_json_buffer) - (size_t)len, "}");
    if (tail < 0 || len + tail >= (int)sizeof(ltl_json_buffer)) {
        return false;
    }
    len += tail;

    file->data = ltl_json_buffer;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return true;
}

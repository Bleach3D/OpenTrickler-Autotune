#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <FreeRTOS.h>
#include <task.h>
#include <u8g2.h>

#include "app.h"
#include "app_state.h"
#include "FloatRingBuffer.h"
#include "mini_12864_module.h"
#include "display.h"
#include "scale.h"
#include "motors.h"
#include "charge_mode.h"
#include "profile.h"
#include "neopixel_led.h"
#include "eeprom.h"
#include "common.h"
#include "pid_autotune.h"

extern "C" void scale_write(const char * command, size_t len);

pid_autotune_t pid_autotune;

extern scale_config_t scale_config;
extern charge_mode_config_t charge_mode_config;
extern neopixel_led_config_t neopixel_led_config;
extern QueueHandle_t encoder_event_queue;

static TaskHandle_t pid_autotune_render_task_handler = NULL;
static TickType_t throw_start_tick = 0;
static bool throw_running = false;
static volatile bool abort_requested = false;

// Tail/margin constants. See fit_profile() for how these are used.
#define PA_COARSE_STOP_MARGIN_GR     0.15f
#define PA_FINE_TAPER_MIN_GR         0.20f
#define PA_FINE_TAPER_TAIL_MULT      3.0f
#define PA_COARSE_STOP_MIN_GR        0.30f
#define PA_MIN_THROW_GR              0.05f
#define PA_BRACKET_USE_FRAC          0.80f
#define PA_SEARCH_STEPS              12
#define PA_LAND_STEPS                6
#define PA_FINE_LAND_FLOW_GPS        0.06f
#define PA_COARSE_FLOW_CAP_GPS       18.0f
#define PA_COARSE_EXTRAP_MULT        1.25f
#define PA_COARSE_STOP_SAFETY_MIN    1.0f
#define PA_COARSE_STOP_SAFETY_MAX    5.0f
#define PA_LAND_SIGMA_MIN            1.0f
#define PA_LAND_SIGMA_MAX            4.0f
#define PA_COARSE_MIN_RUN_S          1.20f
#define PA_COARSE_MAX_RUN_S          4.00f
#define PA_FINE_MIN_RUN_S            3.00f
#define PA_FINE_MAX_RUN_S            12.0f
#define PA_SETTLE_ALLOWANCE_S        0.50f
#define PA_ZERO_TIMEOUT_MS           60000
#define PA_ZERO_RETRY_MS             2500
#define PA_ZERO_MAX_TRIES            12

static const pid_autotune_config_t default_pid_autotune_config = {
    .pid_autotune_config_rev = 0,
    .coarse_target_gr = 8.0f,
    .fine_target_gr = 1.75f,
    .coarse_speed_ceiling = 6.0f,
    .fine_speed_ceiling = 4.0f,
    .confirm_target_gr = 42.5f,
    .time_goal_s = 7.0f,
    .cup_capacity_gr = 250.0f,
    .coarse_stop_safety = 1.5f,
    .land_sigma = 2.0f,
};

static float max_settled_coarse = 0.0f;
static float max_settled_fine = 0.0f;


static void set_message(const char * msg) {
    strncpy(pid_autotune.message, msg, sizeof(pid_autotune.message) - 1);
    pid_autotune.message[sizeof(pid_autotune.message) - 1] = '\0';
}


// ---------------------------------------------------------------------------
// Display -- minimal status while a run is in progress, so the physical
// screen shows something sensible instead of whatever it last had up.
// ---------------------------------------------------------------------------
static void pid_autotune_render_task(void *p) {
    char buf[32];
    u8g2_t *display_handler = get_display_handler();

    while (true) {
        TickType_t last_render_tick = xTaskGetTickCount();
        u8g2_ClearBuffer(display_handler);
        uint8_t screen_width = u8g2_GetDisplayWidth(display_handler);

        u8g2_SetFont(display_handler, u8g2_font_helvB08_tr);
        switch (pid_autotune.state) {
            case PID_AUTOTUNE_STATE_WAIT_FOR_ZERO: snprintf(buf, sizeof(buf), "PID Tune: Zeroing"); break;
            case PID_AUTOTUNE_STATE_COARSE:  snprintf(buf, sizeof(buf), "PID Tune C %d/%d", pid_autotune.throw_idx + 1, PID_AUTOTUNE_THROWS_PER_PHASE); break;
            case PID_AUTOTUNE_STATE_FINE:    snprintf(buf, sizeof(buf), "PID Tune F %d/%d", pid_autotune.throw_idx + 1, PID_AUTOTUNE_THROWS_PER_PHASE); break;
            case PID_AUTOTUNE_STATE_FIT:     snprintf(buf, sizeof(buf), "PID Tune: Fitting"); break;
            case PID_AUTOTUNE_STATE_DONE:    snprintf(buf, sizeof(buf), "PID Tune: Done"); break;
            case PID_AUTOTUNE_STATE_ABORTED: snprintf(buf, sizeof(buf), "PID Tune: Stopped"); break;
            case PID_AUTOTUNE_STATE_ERROR:   snprintf(buf, sizeof(buf), "PID Tune: Error"); break;
            default:                         snprintf(buf, sizeof(buf), "PID Tune"); break;
        }
        u8g2_DrawStr(display_handler, 5, 10, buf);

        if (throw_running) {
            float elapsed = (float)((xTaskGetTickCount() - throw_start_tick) * portTICK_PERIOD_MS) / 1000.0f;
            snprintf(buf, sizeof(buf), "%.1f s", elapsed);
            uint8_t w = u8g2_GetStrWidth(display_handler, buf);
            u8g2_DrawStr(display_handler, screen_width - w - 5, 10, buf);
        }
        u8g2_DrawHLine(display_handler, 0, 13, screen_width);

        if (pid_autotune.state == PID_AUTOTUNE_STATE_DONE && pid_autotune.result_valid) {
            u8g2_SetFont(display_handler, u8g2_font_profont11_tf);
            snprintf(buf, sizeof(buf), "C max %.2f stop %.2f", pid_autotune.result.coarse_max_rps, pid_autotune.result.coarse_stop_threshold);
            u8g2_DrawStr(display_handler, 3, 26, buf);
            snprintf(buf, sizeof(buf), "F max %.2f taper %.2f", pid_autotune.result.fine_max_rps, pid_autotune.result.fine_taper_gr);
            u8g2_DrawStr(display_handler, 3, 38, buf);
            snprintf(buf, sizeof(buf), "Pred %.1fs goal %.1fs", pid_autotune.result.predicted_total_s, pid_autotune.config.time_goal_s);
            u8g2_DrawStr(display_handler, 3, 50, buf);
            u8g2_SetFont(display_handler, u8g2_font_helvR08_tr);
            u8g2_DrawStr(display_handler, 3, 61, "See web GUI. RST to exit");
        }
        else if (pid_autotune.state == PID_AUTOTUNE_STATE_ERROR || pid_autotune.state == PID_AUTOTUNE_STATE_ABORTED) {
            u8g2_SetFont(display_handler, u8g2_font_helvR08_tr);
            u8g2_DrawStr(display_handler, 5, 30, pid_autotune.message);
            u8g2_DrawStr(display_handler, 5, 61, "Press RST to exit");
        }
        else {
            float m = scale_get_current_measurement();
            if (m > -1.0f) {
                float_to_string(buf, m, charge_mode_config.eeprom_charge_mode_data.decimal_places);
            }
            else {
                strcpy(buf, "---");
            }
            u8g2_SetFont(display_handler, u8g2_font_profont22_tf);
            u8g2_DrawStr(display_handler, 26, 35, buf);

            u8g2_SetFont(display_handler, u8g2_font_helvR08_tr);
            if (pid_autotune.state == PID_AUTOTUNE_STATE_COARSE || pid_autotune.state == PID_AUTOTUNE_STATE_FINE) {
                snprintf(buf, sizeof(buf), "%.2f rps", pid_autotune.current_speed);
                u8g2_DrawStr(display_handler, 5, 61, buf);
            }
            uint8_t w = u8g2_GetStrWidth(display_handler, pid_autotune.message);
            u8g2_DrawStr(display_handler, screen_width - w - 3, 61, pid_autotune.message);
        }

        u8g2_SendBuffer(display_handler);
        vTaskDelayUntil(&last_render_tick, pdMS_TO_TICKS(50));
    }
}


// ---------------------------------------------------------------------------
// Scale helpers -- same settle/zero patterns charge_mode.cpp uses elsewhere.
// ---------------------------------------------------------------------------
static bool check_abort(void) {
    if (abort_requested) {
        abort_requested = false;
        pid_autotune.state = PID_AUTOTUNE_STATE_ABORTED;
        set_message("Stopped");
        return false;
    }
    ButtonEncoderEvent_t ev = button_wait_for_input(false);
    if (ev == BUTTON_RST_PRESSED) {
        pid_autotune.state = PID_AUTOTUNE_STATE_ABORTED;
        set_message("Stopped by user");
        return false;
    }
    return true;
}


static bool wait_for_stable(float * settled) {
    FloatRingBuffer data_buffer(8);
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(30000);

    while (true) {
        TickType_t last_tick = xTaskGetTickCount();
        if (!check_abort()) return false;
        if (last_tick > deadline) {
            pid_autotune.state = PID_AUTOTUNE_STATE_ERROR;
            set_message("Scale not settling");
            return false;
        }

        float m;
        if (scale_block_wait_for_next_measurement(300, &m)) {
            data_buffer.enqueue(m);
        }
        if (data_buffer.getCounter() >= 8 &&
            data_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin) {
            *settled = data_buffer.getMean();
            return true;
        }
        vTaskDelayUntil(&last_tick, pdMS_TO_TICKS(200));
    }
}


static bool auto_zero(void) {
    set_message("Zeroing");
    float settled;
    if (!wait_for_stable(&settled)) return false;

    FloatRingBuffer data_buffer(6);
    int tries = 0;
    int samples_since_send = 0;
    TickType_t next_send = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(PA_ZERO_TIMEOUT_MS);

    while (true) {
        TickType_t last_tick = xTaskGetTickCount();
        if (!check_abort()) return false;

        if (last_tick > deadline || tries >= PA_ZERO_MAX_TRIES) {
            pid_autotune.state = PID_AUTOTUNE_STATE_ERROR;
            set_message("Zero failed");
            return false;
        }

        float m;
        if (scale_block_wait_for_next_measurement(300, &m)) {
            data_buffer.enqueue(m);
            if (samples_since_send < 1000) samples_since_send += 1;
        }

        bool steady = data_buffer.getCounter() >= 6 &&
                      data_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin;

        if (steady && samples_since_send >= 6 &&
            fabsf(data_buffer.getMean()) < charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin) {
            return true;
        }

        if (last_tick >= next_send && steady) {
            if ((tries % 2) == 0) {
                if (scale_config.scale_handle->force_zero != NULL) {
                    scale_config.scale_handle->force_zero();
                }
            }
            else {
                const char tare_cmd[] = "T\r\n";
                scale_write(tare_cmd, sizeof(tare_cmd) - 1);
            }
            tries += 1;
            next_send = last_tick + pdMS_TO_TICKS(PA_ZERO_RETRY_MS);
            samples_since_send = 0;
            if (tries > 2) set_message("Zeroing, retrying");
        }

        vTaskDelayUntil(&last_tick, pdMS_TO_TICKS(200));
    }
}


// No cup-dump automation in this version -- abort cleanly rather than
// silently overflowing the cup mid-run.
static bool ensure_cup_room(float expected_gr) {
    if (pid_autotune.cup_load_gr + expected_gr > pid_autotune.config.cup_capacity_gr) {
        pid_autotune.state = PID_AUTOTUNE_STATE_ERROR;
        set_message("Cup full - empty & restart");
        return false;
    }
    return true;
}


// ---------------------------------------------------------------------------
// One throw on one tube: fixed speed, fixed time, then settle. Duration
// based, not weight based -- the scale reads behind the powder, so stopping
// on a reading corrupts both the flow and the lag measurement.
// ---------------------------------------------------------------------------
static bool throw_for_time(motor_select_t motor, float speed, float run_s, float expected_gr,
                           float * max_settled, pid_autotune_throw_t * out) {
    if (!ensure_cup_room(fmaxf(expected_gr, *max_settled) * 1.15f)) return false;

    set_message("Running");
    pid_autotune.current_speed = speed;

    float base = scale_get_current_measurement();
    if (base < -1.0f) base = 0.0f;

    throw_start_tick = xTaskGetTickCount();
    throw_running = true;
    TickType_t stop_target = throw_start_tick + pdMS_TO_TICKS((uint32_t)(run_s * 1000.0f));
    motor_set_speed(motor, speed);

    float latest = base;
    bool ok = true;

    while (xTaskGetTickCount() < stop_target) {
        if (!check_abort()) { ok = false; break; }
        float m;
        if (scale_block_wait_for_next_measurement(150, &m)) {
            latest = m;
        }
    }

    motor_set_speed(motor, 0);
    TickType_t stop_tick = xTaskGetTickCount();
    throw_running = false;
    if (!ok) return false;

    float actual_run_s = (float)((stop_tick - throw_start_tick) * portTICK_PERIOD_MS) / 1000.0f;
    if (actual_run_s <= 0.0f) actual_run_s = 0.001f;
    float stop_weight = latest;

    set_message("Settling");
    float settled;
    if (!wait_for_stable(&settled)) return false;

    float mass = settled - base;
    if (mass < 0.0f) mass = 0.0f;

    out->speed_rps = speed;
    out->time_s = actual_run_s;
    out->stop_weight = stop_weight - base;
    out->settled_weight = mass;
    out->flow_gps = mass / actual_run_s;
    out->tail = mass - out->stop_weight;
    out->lag_s = (out->flow_gps > 0.02f) ? (out->tail / out->flow_gps) : 0.0f;
    if (out->lag_s < 0.0f) out->lag_s = 0.0f;
    if (out->lag_s > 5.0f) out->lag_s = 5.0f;

    if (mass > *max_settled) *max_settled = mass;
    pid_autotune.cup_load_gr += mass;

    if (mass < PA_MIN_THROW_GR) {
        pid_autotune.state = PID_AUTOTUNE_STATE_ERROR;
        char msg[48];
        snprintf(msg, sizeof(msg), "No flow: %.2fgr in %.0fs", mass, actual_run_s);
        set_message(msg);
        return false;
    }

    return auto_zero();
}


// ---------------------------------------------------------------------------
// Fit
// ---------------------------------------------------------------------------
static float fit_flow_per_rps(const pid_autotune_throw_t * throws, int count) {
    double num = 0.0, den = 0.0;
    for (int i = 0; i < count; i += 1) {
        if (throws[i].time_s <= 0.0f) continue;
        num += (double) throws[i].flow_gps * throws[i].speed_rps;
        den += (double) throws[i].speed_rps * throws[i].speed_rps;
    }
    return (den > 0.0) ? (float)(num / den) : 0.0f;
}


static float sample_sd(double sum, double sum_sq, int n) {
    if (n < 2) return 0.0f;
    double mean = sum / n;
    double var = (sum_sq - n * mean * mean) / (n - 1);
    return (var > 0.0) ? (float) sqrt(var) : 0.0f;
}


// Mean and SD of the per-throw lag, per phase -- coarse and fine
// consistently measure different lag, so pooling them inflates the spread
// with the gap between their means rather than just noise.
static void lag_stats(float * coarse_mean, float * coarse_sd, float * fine_mean, float * fine_sd) {
    double c_sum = 0.0, c_sum_sq = 0.0, f_sum = 0.0, f_sum_sq = 0.0;
    int c_n = 0, f_n = 0;
    for (int i = 0; i < PID_AUTOTUNE_THROWS_PER_PHASE; i += 1) {
        if (pid_autotune.coarse[i].lag_s > 0.0f) {
            double v = (double) pid_autotune.coarse[i].lag_s;
            c_sum += v; c_sum_sq += v * v; c_n += 1;
        }
        if (pid_autotune.fine[i].lag_s > 0.0f) {
            double v = (double) pid_autotune.fine[i].lag_s;
            f_sum += v; f_sum_sq += v * v; f_n += 1;
        }
    }
    *coarse_mean = (c_n > 0) ? (float)(c_sum / c_n) : 0.0f;
    *coarse_sd = sample_sd(c_sum, c_sum_sq, c_n);
    *fine_mean = (f_n > 0) ? (float)(f_sum / f_n) : 0.0f;
    *fine_sd = sample_sd(f_sum, f_sum_sq, f_n);
}


// Predict a throw at the confirm target for one coarse level and one fine
// level. This firmware has no lag-compensated stop prediction, so the
// handoff always has to swallow the whole measured tail (see fit_profile).
static void predict_throw(float target, float coarse_flow, float fine_flow_max, float fine_flow_min,
                          float fmax_rps, float fmin_rps,
                          float handoff, float taper_gr, float * coarse_s, float * fine_s) {
    float bulk_rate = coarse_flow + fine_flow_max;
    float bulk_gr = fmaxf(0.0f, target - handoff);
    *coarse_s = (bulk_rate > 0.0f) ? bulk_gr / bulk_rate : 999.0f;

    float full_gr = fmaxf(0.0f, handoff - taper_gr);
    float taper_run = fminf(handoff, taper_gr);
    float t = 999.0f;
    if (fine_flow_max > 0.0f && fine_flow_min > 0.0f) {
        t = full_gr / fine_flow_max;
        float tau = taper_gr / fine_flow_max;
        float e_switch = (fmax_rps > 0.0f) ? taper_gr * fmin_rps / fmax_rps : taper_run;
        if (taper_run > e_switch && e_switch > 0.0f) {
            t += tau * logf(taper_run / e_switch);
            t += e_switch / fine_flow_min;
        }
        else {
            t += taper_run / fine_flow_min;
        }
    }
    *fine_s = t + PA_SETTLE_ALLOWANCE_S;
}


static void fit_profile(void) {
    pid_autotune_result_t * r = &pid_autotune.result;
    memset(r, 0, sizeof(*r));

    float bracket = fmaxf(charge_mode_config.eeprom_charge_mode_data.accept_tolerance_gn, 0.005f);
    float target = pid_autotune.config.confirm_target_gr;
    float goal = pid_autotune.config.time_goal_s;

    r->coarse_k = fit_flow_per_rps(pid_autotune.coarse, PID_AUTOTUNE_THROWS_PER_PHASE);
    r->fine_k = fit_flow_per_rps(pid_autotune.fine, PID_AUTOTUNE_THROWS_PER_PHASE);
    lag_stats(&r->coarse_lag_s, &r->coarse_lag_sd_s, &r->fine_lag_s, &r->fine_lag_sd_s);

    // This firmware has no lag-compensated stop prediction, so the handoff
    // must cover the whole measured tail (lag x flow), not just its
    // scatter -- unlike a firmware that subtracts the predicted tail
    // before stopping, which only needs to cover the *error* in that
    // prediction. coarse_stop_safety is the cushion on top of that.
    float coarse_lag_err = fmaxf(r->coarse_lag_s, 0.05f);
    float fine_lag_err = fmaxf(r->fine_lag_s, 0.05f);

    float coarse_margin_mult = pid_autotune.config.coarse_stop_safety;
    if (coarse_margin_mult < PA_COARSE_STOP_SAFETY_MIN) coarse_margin_mult = PA_COARSE_STOP_SAFETY_MIN;
    if (coarse_margin_mult > PA_COARSE_STOP_SAFETY_MAX) coarse_margin_mult = PA_COARSE_STOP_SAFETY_MAX;

    float c_motor_min = fmaxf(get_motor_min_speed(SELECT_COARSE_TRICKLER_MOTOR), 0.05f);
    float f_motor_min = fmaxf(get_motor_min_speed(SELECT_FINE_TRICKLER_MOTOR), 0.05f);
    float c_hi = fminf(pid_autotune.config.coarse_speed_ceiling, (float) get_motor_max_speed(SELECT_COARSE_TRICKLER_MOTOR));
    float f_hi = fminf(pid_autotune.config.fine_speed_ceiling, (float) get_motor_max_speed(SELECT_FINE_TRICKLER_MOTOR));

    float fmin_land = (r->fine_k > 0.0f) ? PA_FINE_LAND_FLOW_GPS / r->fine_k : f_motor_min;
    float land_budget = PA_BRACKET_USE_FRAC * bracket;
    float land_sigma = pid_autotune.config.land_sigma;
    if (land_sigma < PA_LAND_SIGMA_MIN) land_sigma = PA_LAND_SIGMA_MIN;
    if (land_sigma > PA_LAND_SIGMA_MAX) land_sigma = PA_LAND_SIGMA_MAX;
    float fmin_err = (r->fine_k > 0.0f && fine_lag_err > 0.0f)
                     ? land_budget / (land_sigma * r->fine_k * fine_lag_err) : fmin_land;

    float fmin_hi = fmin_err;
    if (fmin_hi > f_hi) fmin_hi = f_hi;
    if (fmin_hi < f_motor_min) fmin_hi = f_motor_min;
    float fmin_lo = fminf(fmin_land, fmin_hi);
    if (fmin_lo < f_motor_min) fmin_lo = f_motor_min;

    // The fit may only pick a coarse speed the ladder actually characterised, plus a little.
    float c_measured_max = 0.0f;
    for (int i = 0; i < PID_AUTOTUNE_THROWS_PER_PHASE; i += 1) {
        if (pid_autotune.coarse[i].time_s > 0.0f && pid_autotune.coarse[i].speed_rps > c_measured_max) {
            c_measured_max = pid_autotune.coarse[i].speed_rps;
        }
    }
    if (c_measured_max > 0.0f) c_hi = fminf(c_hi, c_measured_max * PA_COARSE_EXTRAP_MULT);
    if (c_hi < c_motor_min) c_hi = c_motor_min;

    float best_c = c_motor_min, best_f = fmin_lo, best_fmin = fmin_lo;
    float best_handoff = 0.0f, best_taper = PA_FINE_TAPER_MIN_GR;
    float best_cs = 0.0f, best_fs = 0.0f, best_total = 1e9f;
    bool found = false;

    float fb_c = c_motor_min, fb_f = fmin_lo, fb_fmin = fmin_lo;
    float fb_handoff = 0.0f, fb_taper = PA_FINE_TAPER_MIN_GR;
    float fb_cs = 0.0f, fb_fs = 0.0f, fb_total = 1e9f;

  for (int li = 0; li < PA_LAND_STEPS; li += 1) {
    float fmin = (PA_LAND_STEPS > 1)
                 ? fmin_lo + (fmin_hi - fmin_lo) * (float) li / (float) (PA_LAND_STEPS - 1)
                 : fmin_lo;
    if (fmin < f_motor_min) fmin = f_motor_min;
    float fine_flow_min = r->fine_k * fmin;

    for (int fi = 1; fi <= PA_SEARCH_STEPS; fi += 1) {
        float fmax = f_hi * (float) fi / (float) PA_SEARCH_STEPS;
        if (fmax < fmin) continue;
        float fine_flow_max = r->fine_k * fmax;

        float taper = fmaxf(PA_FINE_TAPER_MIN_GR, PA_FINE_TAPER_TAIL_MULT * r->fine_lag_s * fine_flow_max);

        for (int ci = 1; ci <= PA_SEARCH_STEPS; ci += 1) {
            float cmax = c_hi * (float) ci / (float) PA_SEARCH_STEPS;
            if (cmax < c_motor_min) continue;
            float coarse_flow = r->coarse_k * cmax;

            float handoff = fmaxf(PA_COARSE_STOP_MIN_GR,
                                  coarse_margin_mult * 3.0f * coarse_flow * coarse_lag_err + PA_COARSE_STOP_MARGIN_GR);
            if (handoff < taper) handoff = taper;
            if (handoff >= 0.5f * target) continue;

            float cs, fs;
            predict_throw(target, coarse_flow, fine_flow_max, fine_flow_min, fmax, fmin,
                          handoff, taper, &cs, &fs);
            float total = cs + fs;

            bool better = !found;
            if (found && handoff < best_handoff - 0.001f) better = true;
            else if (found && handoff < best_handoff + 0.001f) {
                if (fmin < best_fmin - 1e-4f) better = true;
                else if (fmin < best_fmin + 1e-4f && total < best_total) better = true;
            }
            if (total <= goal && better) {
                found = true;
                best_c = cmax; best_f = fmax; best_fmin = fmin;
                best_handoff = handoff; best_taper = taper;
                best_cs = cs; best_fs = fs; best_total = total;
            }
            if (total < fb_total) {
                fb_c = cmax; fb_f = fmax; fb_fmin = fmin;
                fb_handoff = handoff; fb_taper = taper;
                fb_cs = cs; fb_fs = fs; fb_total = total;
            }
        }
    }
  }
    if (!found) {
        best_c = fb_c; best_f = fb_f; best_fmin = fb_fmin;
        best_handoff = fb_handoff; best_taper = fb_taper;
        best_cs = fb_cs; best_fs = fb_fs; best_total = fb_total;
    }
    if (best_total >= 1e9f) {
        best_c = c_motor_min;
        best_fmin = fmin_lo;
        best_f = fmaxf(fmin_lo, f_hi / (float) PA_SEARCH_STEPS);
        best_taper = fmaxf(PA_FINE_TAPER_MIN_GR, PA_FINE_TAPER_TAIL_MULT * r->fine_lag_s * r->fine_k * best_f);
        best_handoff = fmaxf(PA_COARSE_STOP_MIN_GR, best_taper);
        predict_throw(target, r->coarse_k * best_c, r->fine_k * best_f, r->fine_k * best_fmin,
                      best_f, best_fmin, best_handoff, best_taper, &best_cs, &best_fs);
        best_total = best_cs + best_fs;
    }
    found = (best_total <= goal);

    r->fine_min_rps = best_fmin;
    r->fine_max_rps = best_f;
    r->fine_taper_gr = best_taper;
    r->fine_kp = (best_taper > 0.0f) ? (best_f / best_taper) : best_f;

    r->coarse_max_rps = best_c;
    r->coarse_min_rps = fminf(c_motor_min, best_c);
    r->coarse_stop_threshold = best_handoff;

    // Hold full coarse speed until the last half grain before the handoff.
    r->coarse_kp = r->coarse_max_rps / 0.5f;

    r->predicted_coarse_s = best_cs;
    r->predicted_fine_s = best_fs;
    r->predicted_total_s = best_total;
    r->meets_time_goal = found;

    pid_autotune.result_valid = true;
}


bool pid_autotune_apply_to_profile(void) {
    if (!pid_autotune.result_valid) return false;
    pid_autotune_result_t * r = &pid_autotune.result;
    profile_t * p = profile_get_selected();
    if (p == NULL) return false;

    p->coarse_kp = r->coarse_kp;
    p->coarse_ki = 0.0f;
    p->coarse_kd = 0.0f;
    p->coarse_min_flow_speed_rps = r->coarse_min_rps;
    p->coarse_max_flow_speed_rps = r->coarse_max_rps;

    p->fine_kp = r->fine_kp;
    p->fine_ki = 0.0f;
    p->fine_kd = 0.0f;
    p->fine_min_flow_speed_rps = r->fine_min_rps;
    p->fine_max_flow_speed_rps = r->fine_max_rps;

    charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold = r->coarse_stop_threshold;

    pid_autotune.applied_to_profile = true;
    return true;
}


// ---------------------------------------------------------------------------
// Main routine
// ---------------------------------------------------------------------------
void pid_autotune_request_abort(void) {
    abort_requested = true;
}


uint8_t pid_autotune_menu(void) {
    pid_autotune.state = PID_AUTOTUNE_STATE_WAIT_FOR_ZERO;
    pid_autotune.throw_idx = 0;
    pid_autotune.current_speed = 0.0f;
    pid_autotune.result_valid = false;
    pid_autotune.applied_to_profile = false;
    pid_autotune.cup_load_gr = 0.0f;
    abort_requested = false;
    max_settled_coarse = 0.0f;
    max_settled_fine = 0.0f;
    memset(pid_autotune.coarse, 0, sizeof(pid_autotune.coarse));
    memset(pid_autotune.fine, 0, sizeof(pid_autotune.fine));
    set_message("");

    if (pid_autotune_render_task_handler == NULL) {
        UBaseType_t prio = uxTaskPriorityGet(xTaskGetCurrentTaskHandle());
        xTaskCreate(pid_autotune_render_task, "PID Autotune Render", configMINIMAL_STACK_SIZE, NULL, prio - 1, &pid_autotune_render_task_handler);
    }
    else {
        vTaskResume(pid_autotune_render_task_handler);
    }

    neopixel_led_set_colour(charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour,
                            charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour,
                            charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, true);

    motor_enable(SELECT_COARSE_TRICKLER_MOTOR, true);
    motor_enable(SELECT_FINE_TRICKLER_MOTOR, true);

    float coarse_cap = fminf(pid_autotune.config.coarse_speed_ceiling, (float) get_motor_max_speed(SELECT_COARSE_TRICKLER_MOTOR));
    float fine_cap = fminf(pid_autotune.config.fine_speed_ceiling, (float) get_motor_max_speed(SELECT_FINE_TRICKLER_MOTOR));
    float c_motor_min = fmaxf(get_motor_min_speed(SELECT_COARSE_TRICKLER_MOTOR), 0.05f);
    float f_motor_min = fmaxf(get_motor_min_speed(SELECT_FINE_TRICKLER_MOTOR), 0.05f);

    bool ok = auto_zero();

    // Probe throw on each tube first, to size the rest of the ladder.
    pid_autotune_throw_t probe;
    float coarse_k = 0.0f, fine_k = 0.0f;

    if (ok) {
        pid_autotune.state = PID_AUTOTUNE_STATE_COARSE;
        pid_autotune.throw_idx = 0;
        set_message("Probe");
        float probe_speed = fmaxf(c_motor_min, 0.25f * coarse_cap);
        ok = throw_for_time(SELECT_COARSE_TRICKLER_MOTOR, probe_speed, 1.0f, 10.0f, &max_settled_coarse, &probe);
        if (ok) {
            coarse_k = probe.flow_gps / probe_speed;
            if (coarse_k <= 0.0f) coarse_k = 1.0f;
        }
    }

    if (ok) {
        float flow_top = fminf(coarse_k * coarse_cap, PA_COARSE_FLOW_CAP_GPS);
        const float flow_frac[PID_AUTOTUNE_COARSE_SPEED_LEVELS] = {0.20f, 0.45f, 0.70f, 1.00f};
        int per_level = PID_AUTOTUNE_THROWS_PER_PHASE / PID_AUTOTUNE_COARSE_SPEED_LEVELS;

        for (int i = 0; i < PID_AUTOTUNE_THROWS_PER_PHASE && ok; i += 1) {
            pid_autotune.throw_idx = i;
            float want_flow = flow_top * flow_frac[i / per_level];
            float speed = want_flow / coarse_k;
            if (speed < c_motor_min) speed = c_motor_min;
            if (speed > coarse_cap) speed = coarse_cap;
            float flow = coarse_k * speed;

            float run_s = pid_autotune.config.coarse_target_gr / fmaxf(flow, 0.1f);
            if (run_s < PA_COARSE_MIN_RUN_S) run_s = PA_COARSE_MIN_RUN_S;
            if (run_s > PA_COARSE_MAX_RUN_S) run_s = PA_COARSE_MAX_RUN_S;

            ok = throw_for_time(SELECT_COARSE_TRICKLER_MOTOR, speed, run_s, flow * run_s, &max_settled_coarse, &pid_autotune.coarse[i]);
            if (ok && pid_autotune.coarse[i].flow_gps > 0.0f) {
                coarse_k = 0.7f * coarse_k + 0.3f * (pid_autotune.coarse[i].flow_gps / speed);
            }
        }
    }

    if (ok) {
        pid_autotune.state = PID_AUTOTUNE_STATE_FINE;
        pid_autotune.throw_idx = 0;
        set_message("Probe");
        float probe_speed = fmaxf(f_motor_min, 0.40f * fine_cap);
        ok = throw_for_time(SELECT_FINE_TRICKLER_MOTOR, probe_speed, 4.0f, 2.0f, &max_settled_fine, &probe);
        if (ok) {
            fine_k = probe.flow_gps / probe_speed;
            if (fine_k <= 0.0f) fine_k = 0.1f;
        }
    }

    if (ok) {
        float f_flow_top = fine_k * fine_cap;
        const float flow_frac[PID_AUTOTUNE_FINE_SPEED_LEVELS] = {0.15f, 0.45f, 1.00f};
        int per_level = PID_AUTOTUNE_THROWS_PER_PHASE / PID_AUTOTUNE_FINE_SPEED_LEVELS;

        for (int i = 0; i < PID_AUTOTUNE_THROWS_PER_PHASE && ok; i += 1) {
            pid_autotune.throw_idx = i;
            float want_flow = f_flow_top * flow_frac[i / per_level];
            float speed = want_flow / fine_k;
            if (speed < f_motor_min) speed = f_motor_min;
            if (speed > fine_cap) speed = fine_cap;
            float flow = fine_k * speed;

            float run_s = pid_autotune.config.fine_target_gr / fmaxf(flow, 0.01f);
            if (run_s < PA_FINE_MIN_RUN_S) run_s = PA_FINE_MIN_RUN_S;
            if (run_s > PA_FINE_MAX_RUN_S) run_s = PA_FINE_MAX_RUN_S;

            ok = throw_for_time(SELECT_FINE_TRICKLER_MOTOR, speed, run_s, flow * run_s, &max_settled_fine, &pid_autotune.fine[i]);
            if (ok && pid_autotune.fine[i].flow_gps > 0.0f) {
                fine_k = 0.7f * fine_k + 0.3f * (pid_autotune.fine[i].flow_gps / speed);
            }
        }
    }

    if (ok) {
        pid_autotune.state = PID_AUTOTUNE_STATE_FIT;
        set_message("Fitting");
        fit_profile();
        pid_autotune_apply_to_profile();
        vTaskDelay(pdMS_TO_TICKS(800));
    }

    motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
    motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);

    if (ok) {
        pid_autotune.state = PID_AUTOTUNE_STATE_DONE;
        set_message("Done - push to continue");
        neopixel_led_set_colour(charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour,
                                charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour,
                                charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour, true);

        // Applied to the profile in RAM already; the user reviews and
        // saves (or discards) from the web GUI / Profile settings page,
        // same as an AI Tuning suggestion. Wait here only for an exit
        // signal so the on-device screen keeps showing the result.
        while (true) {
            ButtonEncoderEvent_t ev = button_wait_for_input(true);
            if (ev == BUTTON_RST_PRESSED || ev == BUTTON_ENCODER_PRESSED || ev == OVERRIDE_FROM_REST) break;
        }
    }
    else {
        neopixel_led_set_colour(charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour,
                                charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour,
                                charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour, true);
        while (true) {
            ButtonEncoderEvent_t ev = button_wait_for_input(true);
            if (ev == BUTTON_RST_PRESSED || ev == BUTTON_ENCODER_PRESSED || ev == OVERRIDE_FROM_REST) break;
        }
    }

    neopixel_led_set_colour(neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led1_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led2_colour, true);

    vTaskSuspend(pid_autotune_render_task_handler);
    motor_enable(SELECT_COARSE_TRICKLER_MOTOR, false);
    motor_enable(SELECT_FINE_TRICKLER_MOTOR, false);

    exit_state = APP_STATE_DEFAULT;
    return 1;
}


// ---------------------------------------------------------------------------
// Init / config persistence
// ---------------------------------------------------------------------------
static bool config_loaded = false;

static void load_config_from_eeprom(void) {
    if (config_loaded) return;

    pid_autotune.config = default_pid_autotune_config;

    pid_autotune_config_t stored;
    bool ok = eeprom_read(EEPROM_PID_AUTOTUNE_CONFIG_BASE_ADDR, (uint8_t*)&stored, sizeof(stored));
    if (ok && stored.pid_autotune_config_rev == EEPROM_PID_AUTOTUNE_CONFIG_REV) {
        pid_autotune.config = stored;
    }

    pid_autotune.config.coarse_target_gr = fmaxf(fminf(pid_autotune.config.coarse_target_gr, 30.0f), 1.0f);
    pid_autotune.config.fine_target_gr = fmaxf(fminf(pid_autotune.config.fine_target_gr, 10.0f), 0.2f);
    pid_autotune.config.coarse_speed_ceiling = fmaxf(fminf(pid_autotune.config.coarse_speed_ceiling, 10.0f), 0.1f);
    pid_autotune.config.fine_speed_ceiling = fmaxf(fminf(pid_autotune.config.fine_speed_ceiling, 10.0f), 0.05f);
    pid_autotune.config.confirm_target_gr = fmaxf(fminf(pid_autotune.config.confirm_target_gr, 500.0f), 1.0f);
    pid_autotune.config.time_goal_s = fmaxf(fminf(pid_autotune.config.time_goal_s, 60.0f), 1.0f);
    pid_autotune.config.cup_capacity_gr = fmaxf(fminf(pid_autotune.config.cup_capacity_gr, 2000.0f), 20.0f);
    pid_autotune.config.coarse_stop_safety = fmaxf(fminf(pid_autotune.config.coarse_stop_safety, PA_COARSE_STOP_SAFETY_MAX), PA_COARSE_STOP_SAFETY_MIN);
    pid_autotune.config.land_sigma = fmaxf(fminf(pid_autotune.config.land_sigma, PA_LAND_SIGMA_MAX), PA_LAND_SIGMA_MIN);

    config_loaded = true;
}


bool pid_autotune_config_save(void) {
    pid_autotune.config.pid_autotune_config_rev = EEPROM_PID_AUTOTUNE_CONFIG_REV;
    return eeprom_write(EEPROM_PID_AUTOTUNE_CONFIG_BASE_ADDR, (uint8_t*)&pid_autotune.config, sizeof(pid_autotune.config));
}


bool pid_autotune_init(void) {
    memset(&pid_autotune, 0, sizeof(pid_autotune));
    pid_autotune.state = PID_AUTOTUNE_STATE_IDLE;
    load_config_from_eeprom();
    eeprom_register_handler(pid_autotune_config_save);
    return true;
}

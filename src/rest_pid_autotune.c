#include "rest_pid_autotune.h"
#include "pid_autotune.h"
#include "profile.h"
#include "http_rest.h"
#include "common.h"
#include "app.h"
#include "app_state.h"
#include "mini_12864_module.h"
#include "charge_mode.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <FreeRTOS.h>
#include <queue.h>

extern QueueHandle_t encoder_event_queue;
extern charge_mode_config_t charge_mode_config;

static char pid_autotune_json_buffer[3072];

static const char * pid_autotune_state_name(pid_autotune_state_t s) {
    switch (s) {
        case PID_AUTOTUNE_STATE_IDLE:          return "idle";
        case PID_AUTOTUNE_STATE_WAIT_FOR_ZERO:  return "zeroing";
        case PID_AUTOTUNE_STATE_COARSE:         return "coarse";
        case PID_AUTOTUNE_STATE_FINE:           return "fine";
        case PID_AUTOTUNE_STATE_FIT:            return "fitting";
        case PID_AUTOTUNE_STATE_DONE:           return "done";
        case PID_AUTOTUNE_STATE_ABORTED:        return "aborted";
        case PID_AUTOTUNE_STATE_ERROR:          return "error";
        default:                                return "unknown";
    }
}


bool http_rest_pid_autotune_state(struct fs_file *file, int num_params, char *params[], char *values[]) {
    (void)num_params; (void)params; (void)values;

    pid_autotune_result_t * r = &pid_autotune.result;
    int len = snprintf(pid_autotune_json_buffer, sizeof(pid_autotune_json_buffer),
        "%s{\"state\":\"%s\",\"message\":\"%s\","
        "\"throw_idx\":%d,\"throws_per_phase\":%d,\"current_speed\":%.3f,"
        "\"cup_load_gr\":%.2f,\"result_valid\":%s,\"applied_to_profile\":%s,"
        "\"result\":{"
            "\"coarse_k\":%.4f,\"fine_k\":%.4f,"
            "\"coarse_lag_s\":%.3f,\"fine_lag_s\":%.3f,"
            "\"coarse_min_rps\":%.4f,\"coarse_max_rps\":%.4f,\"coarse_kp\":%.4f,"
            "\"fine_min_rps\":%.4f,\"fine_max_rps\":%.4f,\"fine_kp\":%.4f,\"fine_taper_gr\":%.3f,"
            "\"coarse_stop_threshold\":%.3f,"
            "\"predicted_coarse_s\":%.2f,\"predicted_fine_s\":%.2f,\"predicted_total_s\":%.2f,"
            "\"meets_time_goal\":%s"
        "}}",
        http_json_header,
        pid_autotune_state_name(pid_autotune.state),
        pid_autotune.message,
        pid_autotune.throw_idx,
        PID_AUTOTUNE_THROWS_PER_PHASE,
        pid_autotune.current_speed,
        pid_autotune.cup_load_gr,
        pid_autotune.result_valid ? "true" : "false",
        pid_autotune.applied_to_profile ? "true" : "false",
        r->coarse_k, r->fine_k,
        r->coarse_lag_s, r->fine_lag_s,
        r->coarse_min_rps, r->coarse_max_rps, r->coarse_kp,
        r->fine_min_rps, r->fine_max_rps, r->fine_kp, r->fine_taper_gr,
        r->coarse_stop_threshold,
        r->predicted_coarse_s, r->predicted_fine_s, r->predicted_total_s,
        r->meets_time_goal ? "true" : "false");

    if (len < 0 || len >= (int)sizeof(pid_autotune_json_buffer)) return false;
    file->data = pid_autotune_json_buffer;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return true;
}


bool http_rest_pid_autotune_config(struct fs_file *file, int num_params, char *params[], char *values[]) {
    (void)num_params; (void)params; (void)values;

    int len = snprintf(pid_autotune_json_buffer, sizeof(pid_autotune_json_buffer),
        "%s{\"coarse_target_gr\":%.2f,\"fine_target_gr\":%.2f,"
        "\"coarse_speed_ceiling\":%.2f,\"fine_speed_ceiling\":%.2f,"
        "\"confirm_target_gr\":%.2f,\"time_goal_s\":%.2f,\"cup_capacity_gr\":%.1f,"
        "\"coarse_stop_safety\":%.2f,\"land_sigma\":%.2f}",
        http_json_header,
        pid_autotune.config.coarse_target_gr,
        pid_autotune.config.fine_target_gr,
        pid_autotune.config.coarse_speed_ceiling,
        pid_autotune.config.fine_speed_ceiling,
        pid_autotune.config.confirm_target_gr,
        pid_autotune.config.time_goal_s,
        pid_autotune.config.cup_capacity_gr,
        pid_autotune.config.coarse_stop_safety,
        pid_autotune.config.land_sigma);

    if (len < 0 || len >= (int)sizeof(pid_autotune_json_buffer)) return false;
    file->data = pid_autotune_json_buffer;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return true;
}


static float clampf(float v, float lo, float hi) {
    if (!(v >= lo)) return lo;      // also catches NaN, e.g. "?cup_capacity_gr=nan"
    if (v > hi) return hi;
    return v;
}


// Shared by both config_set and start (start accepts the same params so a
// single "Start" button on the web GUI can save-then-run in one request).
static void apply_config_params(int num_params, char *params[], char *values[]) {
    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "coarse_target_gr") == 0) {
            pid_autotune.config.coarse_target_gr = clampf(strtof(values[idx], NULL), 1.0f, 30.0f);
        }
        else if (strcmp(params[idx], "fine_target_gr") == 0) {
            pid_autotune.config.fine_target_gr = clampf(strtof(values[idx], NULL), 0.2f, 10.0f);
        }
        else if (strcmp(params[idx], "coarse_speed_ceiling") == 0) {
            pid_autotune.config.coarse_speed_ceiling = clampf(strtof(values[idx], NULL), 0.1f, 10.0f);
        }
        else if (strcmp(params[idx], "fine_speed_ceiling") == 0) {
            pid_autotune.config.fine_speed_ceiling = clampf(strtof(values[idx], NULL), 0.05f, 10.0f);
        }
        else if (strcmp(params[idx], "confirm_target_gr") == 0) {
            pid_autotune.config.confirm_target_gr = clampf(strtof(values[idx], NULL), 1.0f, 500.0f);
        }
        else if (strcmp(params[idx], "time_goal_s") == 0) {
            pid_autotune.config.time_goal_s = clampf(strtof(values[idx], NULL), 1.0f, 60.0f);
        }
        else if (strcmp(params[idx], "cup_capacity_gr") == 0) {
            pid_autotune.config.cup_capacity_gr = clampf(strtof(values[idx], NULL), 20.0f, 2000.0f);
        }
        else if (strcmp(params[idx], "coarse_stop_safety") == 0) {
            pid_autotune.config.coarse_stop_safety = clampf(strtof(values[idx], NULL), 1.0f, 5.0f);
        }
        else if (strcmp(params[idx], "land_sigma") == 0) {
            pid_autotune.config.land_sigma = clampf(strtof(values[idx], NULL), 1.0f, 4.0f);
        }
    }
}


bool http_rest_pid_autotune_config_set(struct fs_file *file, int num_params, char *params[], char *values[]) {
    apply_config_params(num_params, params, values);
    pid_autotune_config_save();
    return http_rest_pid_autotune_config(file, 0, NULL, NULL);
}


bool http_rest_pid_autotune_start(struct fs_file *file, int num_params, char *params[], char *values[]) {
    apply_config_params(num_params, params, values);
    pid_autotune_config_save();

    bool busy = pid_autotune.state != PID_AUTOTUNE_STATE_IDLE &&
                pid_autotune.state != PID_AUTOTUNE_STATE_DONE &&
                pid_autotune.state != PID_AUTOTUNE_STATE_ABORTED &&
                pid_autotune.state != PID_AUTOTUNE_STATE_ERROR;

    // A normal charge already in progress on the physical menu can't be
    // taken over from here -- same rule REST charge-start already follows.
    bool charge_busy = charge_mode_config.charge_mode_state != CHARGE_MODE_EXIT;

    int len;
    if (busy) {
        len = snprintf(pid_autotune_json_buffer, sizeof(pid_autotune_json_buffer),
            "%s{\"success\":false,\"error\":\"AlreadyRunning\"}", http_json_header);
    }
    else if (charge_busy) {
        len = snprintf(pid_autotune_json_buffer, sizeof(pid_autotune_json_buffer),
            "%s{\"success\":false,\"error\":\"ChargeInProgress\"}", http_json_header);
    }
    else {
        // Still on the previous run's result/error screen: that loop is the
        // one OVERRIDE_FROM_REST wakes, and the menu resets exit_state as
        // soon as pid_autotune_menu() returns, so ask it to loop instead.
        if (pid_autotune_is_active()) {
            pid_autotune_request_restart();
        }
        else {
            exit_state = APP_STATE_ENTER_PID_AUTOTUNE_FROM_REST;
        }
        ButtonEncoderEvent_t button_event = OVERRIDE_FROM_REST;
        if (encoder_event_queue != NULL) {
            (void)xQueueSend(encoder_event_queue, &button_event, pdMS_TO_TICKS(250));
        }
        len = snprintf(pid_autotune_json_buffer, sizeof(pid_autotune_json_buffer),
            "%s{\"success\":true}", http_json_header);
    }

    if (len < 0 || len >= (int)sizeof(pid_autotune_json_buffer)) return false;
    file->data = pid_autotune_json_buffer;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return true;
}


bool http_rest_pid_autotune_action(struct fs_file *file, int num_params, char *params[], char *values[]) {
    bool ok = true;
    const char * error = "";

    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "a") != 0) continue;

        if (strcmp(values[idx], "abort") == 0) {
            pid_autotune_request_abort();
        }
        else if (strcmp(values[idx], "apply") == 0) {
            ok = pid_autotune_apply_to_profile();
            if (!ok) error = "NoResultYet";
        }
        else if (strcmp(values[idx], "save") == 0) {
            ok = pid_autotune_apply_to_profile();
            if (ok) {
                ok = profile_data_save() && charge_mode_config_save();
                if (!ok) error = "SaveFailed";
            }
            else {
                error = "NoResultYet";
            }
        }
        else if (strcmp(values[idx], "exit") == 0) {
            // Let the on-device loop waiting on BUTTON_RST_PRESSED/OVERRIDE_FROM_REST
            // finish and return to the main menu.
            if (encoder_event_queue != NULL) {
                ButtonEncoderEvent_t button_event = OVERRIDE_FROM_REST;
                (void)xQueueSend(encoder_event_queue, &button_event, pdMS_TO_TICKS(250));
            }
        }
        else {
            ok = false;
            error = "UnknownAction";
        }
    }

    int len = snprintf(pid_autotune_json_buffer, sizeof(pid_autotune_json_buffer),
        "%s{\"success\":%s%s%s%s}",
        http_json_header,
        ok ? "true" : "false",
        ok ? "" : ",\"error\":\"",
        ok ? "" : error,
        ok ? "" : "\"");

    if (len < 0 || len >= (int)sizeof(pid_autotune_json_buffer)) return false;
    file->data = pid_autotune_json_buffer;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return true;
}


bool rest_pid_autotune_init(void) {
    rest_register_handler("/rest/pid_autotune_state", http_rest_pid_autotune_state);
    rest_register_handler("/rest/pid_autotune_config", http_rest_pid_autotune_config);
    rest_register_handler("/rest/pid_autotune_config_set", http_rest_pid_autotune_config_set);
    rest_register_handler("/rest/pid_autotune_start", http_rest_pid_autotune_start);
    rest_register_handler("/rest/pid_autotune_action", http_rest_pid_autotune_action);

    printf("PID Autotune REST endpoints registered\n");
    return true;
}

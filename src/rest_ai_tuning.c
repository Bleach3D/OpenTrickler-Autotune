#include "rest_ai_tuning.h"
#include "ai_tuning.h"
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
#include <stdarg.h>
#include <math.h>
#include <FreeRTOS.h>
#include <queue.h>

extern QueueHandle_t encoder_event_queue;
extern charge_mode_config_t charge_mode_config;

static char ai_tuning_json_buffer[24576];
static ai_tuning_session_t rest_session_copy;
static ai_profile_model_t rest_saved_model;
static ai_tuning_history_t rest_history_copy;

static bool send_buffer_overflow_error(struct fs_file *file) {
    static const char overflow_error[] = "HTTP/1.1 500 Internal Server Error\r\n"
        "Content-Type: application/json\r\n\r\n"
        "{\"success\":false,\"error\":\"Response buffer overflow\"}";
    file->data = overflow_error;
    file->len = sizeof(overflow_error) - 1;
    file->index = file->len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return true;
}

static bool finalize_json_response(struct fs_file *file, int len) {
    if (len < 0 || len >= (int)sizeof(ai_tuning_json_buffer)) {
        return send_buffer_overflow_error(file);
    }

    file->data = ai_tuning_json_buffer;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return true;
}

static bool append_jsonf(int *len, const char *fmt, ...) {
    if (len == NULL || *len < 0 || *len >= (int)sizeof(ai_tuning_json_buffer)) {
        return false;
    }

    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(ai_tuning_json_buffer + *len,
                            sizeof(ai_tuning_json_buffer) - (size_t)(*len),
                            fmt,
                            args);
    va_end(args);

    if (written < 0 || written >= (int)(sizeof(ai_tuning_json_buffer) - (size_t)(*len))) {
        return false;
    }

    *len += written;
    return true;
}

static void format_json_float(char *buffer, size_t buffer_len, float value, int decimals) {
    if (buffer == NULL || buffer_len == 0) {
        return;
    }
    if (!isfinite(value)) {
        snprintf(buffer, buffer_len, "null");
        return;
    }
    snprintf(buffer, buffer_len, "%.*f", decimals, value);
}

static const char* ai_tuning_state_to_string(ai_tuning_state_t state) {
    switch (state) {
        case AI_TUNING_IDLE:                    return "idle";
        case AI_TUNING_CHARACTERIZING_COARSE:   return "characterizing_coarse";
        case AI_TUNING_CHARACTERIZING_FINE:     return "characterizing_fine";
        case AI_TUNING_CALIBRATING_COARSE:      return "calibrating_coarse";
        case AI_TUNING_CALIBRATING_FINE:        return "calibrating_fine";
        case AI_TUNING_READY_TO_SAVE:           return "ready_to_save";
        case AI_TUNING_ERROR:                   return "error";
        default:                                return "unknown";
    }
}

static const char* ai_motor_mode_to_string(ai_motor_mode_t mode) {
    switch (mode) {
        case AI_MOTOR_MODE_COARSE_ONLY: return "coarse";
        case AI_MOTOR_MODE_FINE_ONLY:   return "fine";
        default:                        return "normal";
    }
}

static const char* flow_sample_safety_class(const ai_flow_sample_t* sample, bool coarse) {
    if (sample == NULL ||
        sample->speed_rps <= 0.0f ||
        sample->flow_gps <= 0.001f ||
        sample->delivered_weight <= 0.0f) {
        return "invalid";
    }

    float tail = fmaxf(sample->tail_weight, 0.0f);
    float tail_ratio = tail / fmaxf(sample->delivered_weight, 0.001f);
    if (coarse) {
        if (tail > 4.0f || tail_ratio > 0.55f) {
            return "blocked";
        }
        if (tail > 2.5f || tail_ratio > 0.35f) {
            return "caution";
        }
        return "safe";
    }

    if (tail > 0.55f || tail_ratio > 0.42f) {
        return "caution";
    }
    return "safe";
}

static bool append_model_json(int *len, const char *key, const ai_profile_model_t *model) {
    if (model == NULL) {
        return append_jsonf(len, ",\"%s\":null", key);
    }

    if (!append_jsonf(len,
                      ",\"%s\":{"
                      "\"valid\":%s,"
                      "\"enabled\":%s,"
                      "\"coarse_sample_count\":%u,"
                      "\"fine_sample_count\":%u,"
                      "\"fine_recovery_sample_count\":%u,"
                      "\"coarse_flow_slope\":%.6f,"
                      "\"coarse_flow_intercept\":%.6f,"
                      "\"coarse_tail_gn\":%.4f,"
                      "\"coarse_best_speed_rps\":%.4f,"
                      "\"coarse_best_flow_gps\":%.4f,"
                      "\"coarse_trim_speed_rps\":%.4f,"
                      "\"coarse_trim_flow_gps\":%.4f,"
                      "\"coarse_trim_tail_gn\":%.4f,"
                      "\"fine_flow_slope\":%.6f,"
                      "\"fine_flow_intercept\":%.6f,"
                      "\"fine_tail_gn\":%.4f,"
                      "\"fine_best_speed_rps\":%.4f,"
                      "\"fine_best_flow_gps\":%.4f,"
                      "\"fine_recovery_speed_rps\":%.4f,"
                      "\"fine_recovery_flow_gps\":%.4f,"
                      "\"fine_recovery_tail_gn\":%.4f,"
                      "\"recommended_fine_window_gn\":%.4f,"
                      "\"runtime_bias_gn\":%.4f,"
                      "\"fine_fast_flow_gps\":%.4f,"
                      "\"fine_fast_tail_gn\":%.4f,"
                      "\"fine_fast_tail_confidence\":%.4f,"
                      "\"fine_micro_flow_gps\":%.4f,"
                      "\"fine_micro_tail_gn\":%.4f,"
                      "\"fine_micro_tail_confidence\":%.4f,"
                      "\"fine_stop_safety_bias_gn\":%.4f,"
                      "\"estimated_kernel_weight_gn\":%.4f,"
                      "\"kernel_weight_confidence\":%.4f,"
                      "\"user_kernel_weight_gn\":%.4f,"
                      "\"fine_tube_profile_id\":%u,"
                      "\"fine_tube_profile\":\"%s\","
                      "\"machine_calibration\":{"
                      "\"valid\":%s,"
                      "\"coarse_sample_count\":%u,"
                      "\"fine_sample_count\":%u,"
                      "\"scale_sample_period_ms\":%.1f,"
                      "\"coarse_first_response_ms\":%.1f,"
                      "\"coarse_settle_ms\":%.1f,"
                      "\"coarse_tail_avg_gn\":%.4f,"
                      "\"coarse_tail_p95_gn\":%.4f,"
                      "\"coarse_uncertainty_gn\":%.4f,"
                      "\"coarse_open_loop_flow_gps\":%.4f,"
                      "\"trim_open_loop_flow_gps\":%.4f,"
                      "\"fine_first_response_ms\":%.1f,"
                      "\"fine_settle_ms\":%.1f,"
                      "\"fine_tail_avg_gn\":%.4f,"
                      "\"fine_tail_p95_gn\":%.4f,"
                      "\"fine_uncertainty_gn\":%.4f,"
                      "\"fine_open_loop_flow_gps\":%.4f,"
                      "\"micro_open_loop_flow_gps\":%.4f,"
                      "\"recommended_bulk_handoff_gn\":%.4f,"
                      "\"recommended_trim_stop_gn\":%.4f,"
                      "\"post_finish_watch_ms\":%.1f"
                      "}"
                      "}",
                      key,
                      model->valid ? "true" : "false",
                      model->enabled ? "true" : "false",
                      model->coarse_sample_count,
                      model->fine_sample_count,
                      model->fine_recovery_sample_count,
                      model->coarse_flow_slope,
                      model->coarse_flow_intercept,
                      model->coarse_tail_gn,
                      model->coarse_best_speed_rps,
                      model->coarse_best_flow_gps,
                      model->coarse_trim_speed_rps,
                      model->coarse_trim_flow_gps,
                      model->coarse_trim_tail_gn,
                      model->fine_flow_slope,
                      model->fine_flow_intercept,
                      model->fine_tail_gn,
                      model->fine_best_speed_rps,
                      model->fine_best_flow_gps,
                      model->fine_recovery_speed_rps,
                      model->fine_recovery_flow_gps,
                      model->fine_recovery_tail_gn,
                      model->recommended_fine_window_gn,
                      model->runtime_bias_gn,
                      model->fine_fast_flow_gps,
                      model->fine_fast_tail_gn,
                      model->fine_fast_tail_confidence,
                      model->fine_micro_flow_gps,
                      model->fine_micro_tail_gn,
                      model->fine_micro_tail_confidence,
                      model->fine_stop_safety_bias_gn,
                      model->estimated_kernel_weight_gn,
                      model->kernel_weight_confidence,
                      model->user_kernel_weight_gn,
                      model->fine_tube_profile,
                      ai_tuning_fine_tube_profile_to_string((ai_fine_tube_profile_t)model->fine_tube_profile),
                      model->machine.valid ? "true" : "false",
                      model->machine.coarse_sample_count,
                      model->machine.fine_sample_count,
                      model->machine.scale_sample_period_ms,
                      model->machine.coarse_first_response_ms,
                      model->machine.coarse_settle_ms,
                      model->machine.coarse_tail_avg_gn,
                      model->machine.coarse_tail_p95_gn,
                      model->machine.coarse_uncertainty_gn,
                      model->machine.coarse_open_loop_flow_gps,
                      model->machine.trim_open_loop_flow_gps,
                      model->machine.fine_first_response_ms,
                      model->machine.fine_settle_ms,
                      model->machine.fine_tail_avg_gn,
                      model->machine.fine_tail_p95_gn,
                      model->machine.fine_uncertainty_gn,
                      model->machine.fine_open_loop_flow_gps,
                      model->machine.micro_open_loop_flow_gps,
                      model->machine.recommended_bulk_handoff_gn,
                      model->machine.recommended_trim_stop_gn,
                      model->machine.post_finish_watch_ms)) {
        return false;
    }

    return true;
}

static bool append_sample_array(int *len, const char *key, const ai_flow_sample_t *samples, uint8_t count) {
    if (!append_jsonf(len, ",\"%s\":[", key)) {
        return false;
    }

    bool coarse = key != NULL && strstr(key, "coarse") != NULL;
    for (uint8_t idx = 0; idx < count; idx++) {
        float tail_ratio = samples[idx].tail_weight /
                           fmaxf(samples[idx].delivered_weight, 0.001f);
        if (!append_jsonf(len,
                          "%s{\"speed_rps\":%.4f,\"motor_on_time_ms\":%.1f,"
                          "\"delivered_weight\":%.4f,\"tail_weight\":%.4f,"
                          "\"tail_ratio\":%.4f,\"flow_gps\":%.4f,\"safety\":\"%s\"}",
                          idx == 0 ? "" : ",",
                          samples[idx].speed_rps,
                          samples[idx].motor_on_time_ms,
                          samples[idx].delivered_weight,
                          samples[idx].tail_weight,
                          tail_ratio,
                          samples[idx].flow_gps,
                          flow_sample_safety_class(&samples[idx], coarse))) {
            return false;
        }
    }

    return append_jsonf(len, "]");
}

bool http_rest_ai_tuning_start(struct fs_file *file, int num_params,
                               char *params[], char *values[]) {
    int profile_idx = -1;
    float target_weight = 30.0f;

    for (int idx = 0; idx < num_params; idx++) {
        if (strcmp(params[idx], "profile_idx") == 0) {
            profile_idx = atoi(values[idx]);
        }
        else if (strcmp(params[idx], "target") == 0) {
            target_weight = strtof(values[idx], NULL);
        }
    }

    if (pid_autotune_is_active()) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"PID Tuning is running - exit it first\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    if (profile_idx < 0 || profile_idx >= MAX_PROFILE_CNT) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Invalid profile_idx (must be 0-%d)\"}",
            http_json_header, MAX_PROFILE_CNT - 1);
        return finalize_json_response(file, len);
    }

    if (!(target_weight > 0.0f) || !isfinite(target_weight)) {
        target_weight = 30.0f;
    }

    profile_t* profile = profile_select((uint8_t)profile_idx);
    if (profile == NULL) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Failed to select profile\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    if (!ai_tuning_start(profile, target_weight)) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Failed to start AI characterization\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    charge_mode_config.target_charge_weight = target_weight;

    bool entering_charge_mode = !charge_mode_is_menu_active() ||
                                charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT;
    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_ZERO;
    if (entering_charge_mode) {
        exit_state = APP_STATE_ENTER_CHARGE_MODE_FROM_REST;
        ButtonEncoderEvent_t button_event = OVERRIDE_FROM_REST;
        if (encoder_event_queue == NULL ||
            xQueueSend(encoder_event_queue, &button_event, pdMS_TO_TICKS(100)) != pdTRUE) {
            ai_tuning_cancel();
            int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
                "%s{\"success\":false,\"error\":\"Failed to enter charge mode\"}",
                http_json_header);
            return finalize_json_response(file, len);
        }
    }

    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"success\":true,\"message\":\"%s\",\"profile\":\"%s\",\"target_weight\":%.2f}",
        http_json_header,
        entering_charge_mode ? "AI characterization started - entering charge mode"
                             : "AI characterization started",
        profile->name,
        target_weight);
    return finalize_json_response(file, len);
}

bool http_rest_ai_machine_calibration_start(struct fs_file *file, int num_params,
                                            char *params[], char *values[]) {
    int profile_idx = -1;
    float target_weight = 40.0f;

    for (int idx = 0; idx < num_params; idx++) {
        if (strcmp(params[idx], "profile_idx") == 0) {
            profile_idx = atoi(values[idx]);
        }
        else if (strcmp(params[idx], "target") == 0) {
            target_weight = strtof(values[idx], NULL);
        }
    }

    if (pid_autotune_is_active()) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"PID Tuning is running - exit it first\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    if (profile_idx < 0 || profile_idx >= MAX_PROFILE_CNT) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Invalid profile_idx (must be 0-%d)\"}",
            http_json_header, MAX_PROFILE_CNT - 1);
        return finalize_json_response(file, len);
    }

    if (!(target_weight > 0.0f) || !isfinite(target_weight)) {
        target_weight = 40.0f;
    }

    profile_t* profile = profile_select((uint8_t)profile_idx);
    if (profile == NULL) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Failed to select profile\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    if (!ai_tuning_start_machine_calibration(profile, target_weight)) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Run and save powder characterization before machine calibration\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    charge_mode_config.target_charge_weight = target_weight;

    bool entering_charge_mode = !charge_mode_is_menu_active() ||
                                charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT;
    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_ZERO;
    if (entering_charge_mode) {
        exit_state = APP_STATE_ENTER_CHARGE_MODE_FROM_REST;
        ButtonEncoderEvent_t button_event = OVERRIDE_FROM_REST;
        if (encoder_event_queue == NULL ||
            xQueueSend(encoder_event_queue, &button_event, pdMS_TO_TICKS(100)) != pdTRUE) {
            ai_tuning_cancel();
            int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
                "%s{\"success\":false,\"error\":\"Failed to enter charge mode\"}",
                http_json_header);
            return finalize_json_response(file, len);
        }
    }

    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"success\":true,\"message\":\"%s\",\"profile\":\"%s\",\"target_weight\":%.2f}",
        http_json_header,
        entering_charge_mode ? "OpenTrickler machine calibration started - entering charge mode"
                             : "OpenTrickler machine calibration started",
        profile->name,
        target_weight);
    return finalize_json_response(file, len);
}

bool http_rest_ai_tuning_status(struct fs_file *file, int num_params,
                                char *params[], char *values[]) {
    int requested_profile_idx = -1;
    for (int idx = 0; idx < num_params; idx++) {
        if (strcmp(params[idx], "profile_idx") == 0) {
            requested_profile_idx = atoi(values[idx]);
        }
    }

    memset(&rest_session_copy, 0, sizeof(rest_session_copy));
    if (!ai_tuning_get_session_copy(&rest_session_copy)) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"AI tuning state busy\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    memset(&rest_saved_model, 0, sizeof(rest_saved_model));
    bool is_active = (rest_session_copy.state == AI_TUNING_CHARACTERIZING_COARSE ||
                      rest_session_copy.state == AI_TUNING_CHARACTERIZING_FINE ||
                      rest_session_copy.state == AI_TUNING_CALIBRATING_COARSE ||
                      rest_session_copy.state == AI_TUNING_CALIBRATING_FINE);
    bool is_complete = (rest_session_copy.state == AI_TUNING_READY_TO_SAVE);
    uint8_t profile_idx = (requested_profile_idx >= 0 && requested_profile_idx < MAX_PROFILE_CNT)
        ? (uint8_t)requested_profile_idx
        : ((is_active || is_complete) && rest_session_copy.target_profile_idx < MAX_PROFILE_CNT
              ? rest_session_copy.target_profile_idx
              : (uint8_t)profile_get_selected_idx());
    bool have_saved_model = ai_tuning_get_enabled_model_copy(profile_idx, &rest_saved_model);
    ai_tuning_plan_t plan = {};
    bool have_plan = ai_tuning_get_active_plan(&plan);
    uint8_t progress = ai_tuning_get_progress_percent();

    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"state\":\"%s\",\"is_active\":%s,\"is_complete\":%s,\"drops_completed\":%u,\"drops_max\":%u,\"progress_percent\":%u,\"profile_idx\":%u,\"requested_target_weight\":%.4f,\"status_message\":\"%s\",\"error_message\":\"%s\"",
        http_json_header,
        ai_tuning_state_to_string(rest_session_copy.state),
        is_active ? "true" : "false",
        is_complete ? "true" : "false",
        rest_session_copy.drops_completed,
        rest_session_copy.total_samples_planned,
        progress,
        profile_idx,
        rest_session_copy.requested_target_weight,
        rest_session_copy.status_message,
        rest_session_copy.error_message);

    if (len < 0 || len >= (int)sizeof(ai_tuning_json_buffer)) {
        return send_buffer_overflow_error(file);
    }

    if (have_plan && plan.valid) {
        if (!append_jsonf(&len,
                          ",\"current_plan\":{\"motor_mode\":\"%s\",\"speed_rps\":%.4f,\"motor_on_time_ms\":%.1f,\"target_weight\":%.4f,\"stage_budget_used_gn\":%.4f,\"stage_budget_limit_gn\":%.4f,\"sample_index\":%u,\"total_samples\":%u,\"description\":\"%s\"}",
                          ai_motor_mode_to_string(plan.motor_mode),
                          plan.speed_rps,
                          plan.motor_on_time_ms,
                          plan.target_weight,
                          plan.stage_budget_used_gn,
                          plan.stage_budget_limit_gn,
                          plan.sample_index,
                          plan.total_samples,
                          plan.description)) {
            return send_buffer_overflow_error(file);
        }
    }

    if (rest_session_copy.drops_completed > 0) {
        uint8_t last_idx = (rest_session_copy.drop_write_idx + AI_TUNING_DROP_BUF_SIZE - 1) % AI_TUNING_DROP_BUF_SIZE;
        const ai_drop_telemetry_t* last = &rest_session_copy.drops[last_idx];
        if (!append_jsonf(&len,
                          ",\"last_drop\":{\"sample_number\":%u,\"motor_mode\":\"%s\",\"speed_rps\":%.4f,\"motor_on_time_ms\":%.1f,\"start_weight\":%.4f,\"stop_weight\":%.4f,\"final_weight\":%.4f,\"delivered_weight\":%.4f,\"tail_weight\":%.4f,\"target_weight\":%.4f,\"overthrow\":%.4f,\"coarse_time_ms\":%.1f,\"fine_time_ms\":%.1f,\"total_time_ms\":%.1f,\"first_response_time_ms\":%.1f,\"settle_time_ms\":%.1f,\"scale_sample_period_ms\":%.1f}",
                          last->sample_number,
                          ai_motor_mode_to_string(last->motor_mode),
                          last->speed_rps,
                          last->motor_on_time_ms,
                          last->start_weight,
                          last->stop_weight,
                          last->final_weight,
                          last->delivered_weight,
                          last->tail_weight,
                          last->target_weight,
                          last->overthrow,
                          last->coarse_time_ms,
                          last->fine_time_ms,
                          last->total_time_ms,
                          last->first_response_time_ms,
                          last->settle_time_ms,
                          last->scale_sample_period_ms)) {
            return send_buffer_overflow_error(file);
        }
    }

    if (!append_model_json(&len, "working_model", &rest_session_copy.working_model)) {
        return send_buffer_overflow_error(file);
    }

    if (have_saved_model) {
        if (!append_model_json(&len, "saved_model", &rest_saved_model)) {
            return send_buffer_overflow_error(file);
        }
    }
    else if (!append_jsonf(&len, ",\"saved_model\":null")) {
        return send_buffer_overflow_error(file);
    }

    if (!append_jsonf(&len, "}")) {
        return send_buffer_overflow_error(file);
    }

    return finalize_json_response(file, len);
}

bool http_rest_ai_tuning_apply(struct fs_file *file, int num_params,
                               char *params[], char *values[]) {
    (void)num_params;
    (void)params;
    (void)values;

    if (!ai_tuning_apply_params()) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"No completed AI model ready to save\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"success\":true,\"message\":\"AI model saved for this profile\"}",
        http_json_header);
    return finalize_json_response(file, len);
}

bool http_rest_ai_tuning_cancel(struct fs_file *file, int num_params,
                                char *params[], char *values[]) {
    (void)num_params;
    (void)params;
    (void)values;

    if (!ai_tuning_cancel()) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Failed to cancel AI tuning\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"success\":true,\"message\":\"AI tuning cancelled\"}",
        http_json_header);
    return finalize_json_response(file, len);
}

bool http_rest_ai_tuning_history(struct fs_file *file, int num_params,
                                 char *params[], char *values[]) {
    int requested_profile_idx = (int)profile_get_selected_idx();
    for (int idx = 0; idx < num_params; idx++) {
        if (strcmp(params[idx], "profile_idx") == 0) {
            requested_profile_idx = atoi(values[idx]);
        }
    }

    if (requested_profile_idx < 0 || requested_profile_idx >= MAX_PROFILE_CNT) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Invalid profile_idx\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    memset(&rest_history_copy, 0, sizeof(rest_history_copy));
    if (!ai_tuning_get_history_copy(&rest_history_copy)) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"AI tuning history busy\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    const ai_profile_model_t* model = &rest_history_copy.models[requested_profile_idx];
    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"profile_idx\":%d,\"observation_count\":%u",
        http_json_header,
        requested_profile_idx,
        rest_history_copy.observation_count);
    if (len < 0 || len >= (int)sizeof(ai_tuning_json_buffer)) {
        return send_buffer_overflow_error(file);
    }

    if (!append_model_json(&len, "model", model)) {
        return send_buffer_overflow_error(file);
    }
    if (!append_sample_array(&len, "coarse_samples", model->coarse_samples, model->coarse_sample_count)) {
        return send_buffer_overflow_error(file);
    }
    if (!append_sample_array(&len, "fine_samples", model->fine_samples, model->fine_sample_count)) {
        return send_buffer_overflow_error(file);
    }
    if (!append_sample_array(&len,
                             "fine_recovery_samples",
                             model->fine_recovery_samples,
                             model->fine_recovery_sample_count)) {
        return send_buffer_overflow_error(file);
    }

    ai_runtime_profile_stats_t runtime_stats;
    memset(&runtime_stats, 0, sizeof(runtime_stats));
    bool runtime_stats_valid =
        ai_tuning_get_runtime_profile_stats((uint8_t)requested_profile_idx,
                                            &runtime_stats);
    if (!append_jsonf(&len,
                      ",\"runtime_stats\":{"
                      "\"valid\":%s,\"observation_count\":%u,"
                      "\"coarse_tail_count\":%u,\"coarse_tail_mean_gn\":%.4f,"
                      "\"coarse_tail_sd_gn\":%.4f,\"coarse_tail_p95_gn\":%.4f,"
                      "\"coarse_tail_max_gn\":%.4f,\"fine_tail_count\":%u,"
                      "\"fine_tail_mean_gn\":%.4f,\"fine_tail_sd_gn\":%.4f,"
                      "\"fine_tail_p90_gn\":%.4f,\"fine_tail_p95_gn\":%.4f,"
                      "\"fine_landing_count\":%u,\"fine_landing_error_mean_gn\":%.4f,"
                      "\"fine_landing_error_p90_gn\":%.4f,"
                      "\"fast_finish_count\":%u,\"fast_finish_tail_count\":%u,"
                      "\"fast_finish_tail_p90_gn\":%.4f,\"fast_finish_tail_p95_gn\":%.4f,"
                      "\"fast_finish_over_rate\":%.4f,\"recovery_phase_count\":%u,"
                      "\"recovery_over_rate\":%.4f,\"coarse_late_count\":%u,"
                      "\"coarse_late_rate\":%.4f,\"recovery_count\":%u,"
                      "\"recovery_use_rate\":%.4f,\"median_recovery_motor_ms\":%.1f,"
                      "\"recovery_flow_count\":%u,\"recovery_flow_p25_gps\":%.4f,"
                      "\"recovery_flow_median_gps\":%.4f,"
                      "\"median_total_time_ms\":%.1f,\"over_rate\":%.4f,"
                      "\"under_rate\":%.4f}",
                      runtime_stats_valid ? "true" : "false",
                      runtime_stats.observation_count,
                      runtime_stats.coarse_tail_count,
                      runtime_stats.coarse_tail_mean_gn,
                      runtime_stats.coarse_tail_sd_gn,
                      runtime_stats.coarse_tail_p95_gn,
                      runtime_stats.coarse_tail_max_gn,
                      runtime_stats.fine_tail_count,
                      runtime_stats.fine_tail_mean_gn,
                      runtime_stats.fine_tail_sd_gn,
                      runtime_stats.fine_tail_p90_gn,
                      runtime_stats.fine_tail_p95_gn,
                      runtime_stats.fine_landing_count,
                      runtime_stats.fine_landing_error_mean_gn,
                      runtime_stats.fine_landing_error_p90_gn,
                      runtime_stats.fast_finish_count,
                      runtime_stats.fast_finish_tail_count,
                      runtime_stats.fast_finish_tail_p90_gn,
                      runtime_stats.fast_finish_tail_p95_gn,
                      runtime_stats.fast_finish_over_rate,
                      runtime_stats.recovery_phase_count,
                      runtime_stats.recovery_over_rate,
                      runtime_stats.coarse_late_count,
                      runtime_stats.coarse_late_rate,
                      runtime_stats.recovery_count,
                      runtime_stats.recovery_use_rate,
                      runtime_stats.median_recovery_motor_ms,
                      runtime_stats.recovery_flow_count,
                      runtime_stats.recovery_flow_p25_gps,
                      runtime_stats.recovery_flow_median_gps,
                      runtime_stats.median_total_time_ms,
                      runtime_stats.over_rate,
                      runtime_stats.under_rate)) {
        return send_buffer_overflow_error(file);
    }

    if (!append_jsonf(&len, ",\"observations\":[")) {
        return send_buffer_overflow_error(file);
    }

    int oldest_idx = (rest_history_copy.observation_count >= AI_RUNTIME_OBSERVATION_COUNT)
        ? rest_history_copy.observation_next_idx
        : 0;
    bool first_observation = true;
    float avg_error = 0.0f;
    float avg_time = 0.0f;
    uint16_t matching_count = 0;

    for (int i = 0; i < rest_history_copy.observation_count; i++) {
        int obs_idx = (oldest_idx + i) % AI_RUNTIME_OBSERVATION_COUNT;
        const ai_runtime_observation_t* obs = &rest_history_copy.observations[obs_idx];
        if (obs->profile_idx != (uint8_t)requested_profile_idx) {
            continue;
        }

        avg_error += obs->final_error_gn;
        avg_time += obs->total_time_ms;
        matching_count++;

        char coarse_stop[16];
        char after_coarse[16];
        char coarse_tail[16];
        char fine_stop[16];
        char after_fine[16];
        char fine_tail[16];
        char post_finish_peak[16];
        char recovery_start[16];
        char recovery_end[16];
        char recovery_motor_on[16];
        format_json_float(coarse_stop, sizeof(coarse_stop), obs->coarse_stop_weight_gn, 4);
        format_json_float(after_coarse, sizeof(after_coarse), obs->after_coarse_settle_gn, 4);
        format_json_float(coarse_tail, sizeof(coarse_tail), obs->observed_coarse_tail_gn, 4);
        format_json_float(fine_stop, sizeof(fine_stop), obs->fine_stop_weight_gn, 4);
        format_json_float(after_fine, sizeof(after_fine), obs->after_fine_settle_gn, 4);
        format_json_float(fine_tail, sizeof(fine_tail), obs->observed_fine_tail_gn, 4);
        format_json_float(post_finish_peak, sizeof(post_finish_peak), obs->post_finish_peak_weight_gn, 4);
        format_json_float(recovery_start, sizeof(recovery_start), obs->recovery_start_weight_gn, 4);
        format_json_float(recovery_end, sizeof(recovery_end), obs->recovery_end_weight_gn, 4);
        format_json_float(recovery_motor_on, sizeof(recovery_motor_on), obs->recovery_motor_on_ms, 1);

        if (!append_jsonf(&len,
                          "%s{\"target_weight\":%.4f,\"final_error_gn\":%.4f,\"total_time_ms\":%.1f,"
                          "\"coarse_stop_weight_gn\":%s,\"after_coarse_settle_gn\":%s,"
                          "\"observed_coarse_tail_gn\":%s,\"fine_stop_weight_gn\":%s,"
                          "\"after_fine_settle_gn\":%s,\"observed_fine_tail_gn\":%s,"
                          "\"post_finish_peak_weight_gn\":%s,\"recovery_start_weight_gn\":%s,"
                          "\"recovery_end_weight_gn\":%s,\"recovery_motor_on_ms\":%s,"
                          "\"recovery_stall_count\":%u,\"recovery_exit_reason\":%u}",
                          first_observation ? "" : ",",
                          obs->target_weight,
                          obs->final_error_gn,
                          obs->total_time_ms,
                          coarse_stop,
                          after_coarse,
                          coarse_tail,
                          fine_stop,
                          after_fine,
                          fine_tail,
                          post_finish_peak,
                          recovery_start,
                          recovery_end,
                          recovery_motor_on,
                          obs->recovery_stall_count,
                          obs->recovery_exit_reason)) {
            return send_buffer_overflow_error(file);
        }
        first_observation = false;
    }

    if (!append_jsonf(&len, "]")) {
        return send_buffer_overflow_error(file);
    }

    if (matching_count > 0) {
        avg_error /= matching_count;
        avg_time /= matching_count;
    }

    if (!append_jsonf(&len,
                      ",\"summary\":{\"matching_observations\":%u,\"avg_error_gn\":%.4f,\"avg_time_ms\":%.1f}}",
                      matching_count,
                      avg_error,
                      avg_time)) {
        return send_buffer_overflow_error(file);
    }

    return finalize_json_response(file, len);
}

bool http_rest_ai_tuning_apply_refined(struct fs_file *file, int num_params,
                                       char *params[], char *values[]) {
    (void)num_params;
    (void)params;
    (void)values;

    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"success\":false,\"error\":\"Deprecated endpoint. Use /rest/ai_tuning_apply to save the learned model.\"}",
        http_json_header);
    return finalize_json_response(file, len);
}

bool http_rest_ai_tuning_clear_history(struct fs_file *file, int num_params,
                                       char *params[], char *values[]) {
    (void)num_params;
    (void)params;
    (void)values;

    ai_tuning_clear_history();
    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"success\":true,\"message\":\"AI history and saved models cleared\"}",
        http_json_header);
    return finalize_json_response(file, len);
}

bool http_rest_ai_tuning_config_get(struct fs_file *file, int num_params,
                                    char *params[], char *values[]) {
    (void)num_params;
    (void)params;
    (void)values;

    ai_tuning_config_t* cfg = ai_tuning_get_config();
    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"coarse_budget_gn\":%.2f,\"fine_budget_gn\":%.2f,\"coarse_sample_count\":%u,\"fine_sample_count\":%u,\"coarse_sample_target_gn\":%.2f,\"fine_sample_target_gn\":%.2f,\"noise_margin\":%.4f,\"time_cost_weight\":%.4f,\"error_cost_weight\":%.4f,\"safety_margin_pct\":%.1f}",
        http_json_header,
        cfg->coarse_budget_gn,
        cfg->fine_budget_gn,
        cfg->coarse_sample_count,
        cfg->fine_sample_count,
        cfg->coarse_sample_target_gn,
        cfg->fine_sample_target_gn,
        cfg->noise_margin,
        cfg->time_cost_weight,
        cfg->error_cost_weight,
        cfg->safety_margin_pct);
    return finalize_json_response(file, len);
}

bool http_rest_ai_tuning_config_set(struct fs_file *file, int num_params,
                                    char *params[], char *values[]) {
    // Sample counts and budgets are read live by the running session (stage
    // limits, sample plans), so changing them mid-run would reshape a
    // characterization half way through.
    if (ai_tuning_is_active()) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"AI tuning is running - wait for it to finish or cancel it first\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    ai_tuning_config_t* cfg = ai_tuning_get_config();

    for (int idx = 0; idx < num_params; idx++) {
        if (strcmp(params[idx], "coarse_budget_gn") == 0) {
            cfg->coarse_budget_gn = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "fine_budget_gn") == 0) {
            cfg->fine_budget_gn = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "coarse_sample_count") == 0) {
            cfg->coarse_sample_count = (uint8_t)atoi(values[idx]);
        }
        else if (strcmp(params[idx], "fine_sample_count") == 0) {
            cfg->fine_sample_count = (uint8_t)atoi(values[idx]);
        }
        else if (strcmp(params[idx], "coarse_sample_target_gn") == 0) {
            cfg->coarse_sample_target_gn = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "fine_sample_target_gn") == 0) {
            cfg->fine_sample_target_gn = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "noise_margin") == 0) {
            cfg->noise_margin = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "time_cost_weight") == 0) {
            cfg->time_cost_weight = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "error_cost_weight") == 0) {
            cfg->error_cost_weight = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "safety_margin_pct") == 0) {
            cfg->safety_margin_pct = strtof(values[idx], NULL);
        }
    }

    cfg->coarse_budget_gn = fmaxf(20.0f, fminf(500.0f, cfg->coarse_budget_gn));
    cfg->fine_budget_gn = fmaxf(5.0f, fminf(150.0f, cfg->fine_budget_gn));
    cfg->coarse_sample_count = (uint8_t)fmaxf(2.0f, fminf((float)AI_TUNING_STAGE_SAMPLE_COUNT, (float)cfg->coarse_sample_count));
    cfg->fine_sample_count = (uint8_t)fmaxf(2.0f, fminf((float)AI_TUNING_STAGE_SAMPLE_COUNT, (float)cfg->fine_sample_count));
    cfg->coarse_sample_target_gn = fmaxf(2.0f, fminf(50.0f, cfg->coarse_sample_target_gn));
    cfg->fine_sample_target_gn = fmaxf(0.2f, fminf(10.0f, cfg->fine_sample_target_gn));
    cfg->noise_margin = fmaxf(0.005f, fminf(0.25f, cfg->noise_margin));
    cfg->time_cost_weight = fmaxf(0.1f, fminf(20.0f, cfg->time_cost_weight));
    cfg->error_cost_weight = fmaxf(0.1f, fminf(50.0f, cfg->error_cost_weight));
    cfg->safety_margin_pct = fmaxf(0.0f, fminf(50.0f, cfg->safety_margin_pct));

    ai_tuning_save_config();
    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"success\":true,\"message\":\"AI configuration saved\"}",
        http_json_header);
    return finalize_json_response(file, len);
}


/*
 * Suggested PID baseline.
 *
 * Turns what characterization physically measured into the profile fields the
 * PID controller actually uses. Nothing here is applied automatically: the UI
 * shows these next to your current values so you can accept the ones you want
 * and hand-tune from there.
 *
 * The derivation, so a number you disagree with can be traced:
 *
 *   The plant is  dW/dt = k * speed,  where k is grains per second per rps,
 *   measured directly as (best_flow_gps / best_speed_rps).
 *
 *   For a first-order approach to target with time constant tau, the wanted
 *   command is  speed = error / (tau * k),  and since the PID computes
 *   speed = Kp * error, that gives  Kp = 1 / (tau * k).
 *
 *   That value alone can be too aggressive: it says nothing about how much
 *   powder is physically in transit (auger flights, chute) at the instant
 *   the phase crosses its own stop threshold. A faster motor at that instant
 *   carries more in transit, so the real tail ends up bigger than the tail
 *   the stop threshold assumed -- which shows up as a systematic overthrow.
 *   Kp is therefore also capped so the *commanded speed at the stop
 *   threshold* (Kp * threshold) does not exceed the speed the tail was
 *   actually characterized at (the trim/recovery speed used for the slow,
 *   final part of characterization).
 *
 *   Ki and Kd are suggested as 0 on purpose. With an A&D FX-120i the reading
 *   is quantised to ~0.0154 gn and lags by roughly a second: a derivative
 *   term mostly amplifies that quantisation, and an integral term winds up
 *   during the long fine phase. Add them by hand later if your setup wants
 *   them, but zero is the honest starting point.
 *
 *   The stop thresholds matter more than the gains for final accuracy,
 *   because the scale lag means the decision of *when to stop* dominates:
 *     coarse stop = measured coarse tail (p95) + one scale period of flow
 *     fine stop   = measured fine tail (p95), floored at one kernel
 *
 *   Both the gains and the thresholds prefer *live* evidence over the
 *   original characterization when enough of it exists: if "ML Data
 *   Collection" is on, every normal PID charge is logged, and once a handful
 *   have landed this function uses their observed tail and over/under rate
 *   instead of (or blended with) the one-off characterization numbers. This
 *   is what lets the suggestion actually come down after a few overthrows,
 *   and what re-anchors it to current hardware after something physical
 *   changes (e.g. a motor microstep setting) without needing a full
 *   re-characterization. Characterization measures speed-vs-flow once and
 *   precisely; live data measures how the real controller with real
 *   hardware actually finishes charges, and the second is what you feel.
 *
 *   Finally, "safety_margin_pct" (Settings > AI Tuning > Suggested PID
 *   Baseline, 0-50%, default 0) is a manual, user-set multiplier applied on
 *   top of everything above: it widens both stop thresholds and lets the Kp
 *   cap tighten to match. It exists for the case where none of the above
 *   margins turn out to be enough for a particular setup -- widening it can
 *   only make the controller stop earlier, never later, so it is always a
 *   safe response to a reported overthrow, independent of waiting for more
 *   logged throws to re-anchor the live-data margin.
 */
bool http_rest_ai_suggestions(struct fs_file *file, int num_params,
                              char *params[], char *values[]) {
    int profile_idx = (int)profile_get_selected_idx();
    for (int idx = 0; idx < num_params; idx++) {
        if (strcmp(params[idx], "profile_idx") == 0) {
            profile_idx = atoi(values[idx]);
        }
    }
    if (profile_idx < 0 || profile_idx >= MAX_PROFILE_CNT) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Invalid profile_idx\"}", http_json_header);
        return finalize_json_response(file, len);
    }

    ai_profile_model_t model;
    memset(&model, 0, sizeof(model));
    bool have_model = ai_tuning_get_profile_model_copy((uint8_t)profile_idx, &model) && model.valid;

    if (!have_model) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":true,\"available\":false,"
            "\"message\":\"Run characterization on this profile to generate a suggested baseline.\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    ai_runtime_profile_stats_t runtime_stats;
    memset(&runtime_stats, 0, sizeof(runtime_stats));
    bool have_runtime = ai_tuning_get_runtime_profile_stats((uint8_t)profile_idx, &runtime_stats) &&
                        runtime_stats.valid;
    // Below this many logged PID throws, live numbers are still mostly
    // sampling noise; keep trusting characterization until there is enough
    // evidence to override it.
    bool runtime_trustworthy = have_runtime && runtime_stats.observation_count >= 4;

    // --- measured plant gains, grains per second per rps ---
    float k_coarse = (model.coarse_best_speed_rps > 0.01f)
        ? model.coarse_best_flow_gps / model.coarse_best_speed_rps : 0.0f;
    float k_fine = (model.fine_best_speed_rps > 0.001f)
        ? model.fine_best_flow_gps / model.fine_best_speed_rps : 0.0f;

    // --- time constants from the configured phase time targets ---
    ai_tuning_config_t* cfg = ai_tuning_get_config();
    float coarse_target_s = charge_mode_config.eeprom_charge_mode_data.coarse_time_target_ms / 1000.0f;
    float total_target_s = charge_mode_config.eeprom_charge_mode_data.total_time_target_ms / 1000.0f;
    if (coarse_target_s < 1.0f) coarse_target_s = 6.0f;
    if (total_target_s < 2.0f) total_target_s = 20.0f;
    float fine_target_s = fmaxf(2.0f, total_target_s - coarse_target_s);

    // Aim to cover the phase in roughly three time constants.
    float tau_coarse = fmaxf(0.5f, coarse_target_s / 3.0f);
    float tau_fine = fmaxf(0.5f, fine_target_s / 3.0f);

    float kp_coarse = (k_coarse > 0.001f) ? 1.0f / (tau_coarse * k_coarse) : 0.0f;
    float kp_fine = (k_fine > 0.0001f) ? 1.0f / (tau_fine * k_fine) : 0.0f;

    // --- stop thresholds: prefer live-observed tail once there is enough of it ---
    float scale_period_s = (model.machine.valid && model.machine.scale_sample_period_ms > 1.0f)
        ? model.machine.scale_sample_period_ms / 1000.0f : 0.15f;

    float coarse_tail_characterized = (model.machine.valid && model.machine.coarse_tail_p95_gn > 0.0f)
        ? model.machine.coarse_tail_p95_gn : model.coarse_tail_gn;
    float fine_tail_characterized = (model.machine.valid && model.machine.fine_tail_p95_gn > 0.0f)
        ? model.machine.fine_tail_p95_gn : model.fine_tail_gn;

    float coarse_tail = coarse_tail_characterized;
    float fine_tail = fine_tail_characterized;
    bool used_live_coarse_tail = false;
    bool used_live_fine_tail = false;
    if (runtime_trustworthy) {
        if (runtime_stats.coarse_tail_count >= 4 && runtime_stats.coarse_tail_p95_gn > 0.0f) {
            // The larger of the two: live evidence should only ever widen
            // the safety margin relative to the lab measurement, never
            // narrow it on the strength of a still-small sample.
            coarse_tail = fmaxf(coarse_tail_characterized, runtime_stats.coarse_tail_p95_gn);
            used_live_coarse_tail = (coarse_tail > coarse_tail_characterized + 0.001f);
        }
        if (runtime_stats.fine_tail_count >= 4 && runtime_stats.fine_tail_p95_gn > 0.0f) {
            fine_tail = fmaxf(fine_tail_characterized, runtime_stats.fine_tail_p95_gn);
            used_live_fine_tail = (fine_tail > fine_tail_characterized + 0.001f);
        }
    }

    float kernel = ai_tuning_effective_kernel_weight_gn(&model, NULL);

    // Coarse must stop early enough that its own tail plus one scale period
    // of continued flow does not carry past target.
    float coarse_stop = coarse_tail + model.coarse_trim_flow_gps * scale_period_s;
    if (!isfinite(coarse_stop) || coarse_stop <= 0.0f) {
        coarse_stop = fmaxf(0.30f, coarse_tail);
    }
    coarse_stop = fmaxf(0.10f, fminf(5.0f, coarse_stop));

    float fine_stop = fmaxf(fine_tail, kernel > 0.0f ? kernel : cfg->noise_margin);
    fine_stop = fmaxf(0.010f, fminf(0.50f, fine_stop));

    float accept_tol = fmaxf(kernel > 0.0f ? kernel : 0.0f, 0.0154f);
    accept_tol = fmaxf(0.008f, fminf(0.20f, accept_tol));

    // --- user-dialed extra safety margin (Settings > AI Tuning > Suggested
    // PID Baseline). 0 by default, leaving everything above unchanged. This
    // widens both stop thresholds on top of whatever characterization/live
    // data already derived, *before* the Kp cap below, so the cap tightens
    // to match automatically rather than needing a second, separate
    // adjustment. A wider stop threshold can only make the controller stop
    // earlier -- it is not possible for this setting to increase overthrow
    // risk, only reduce it (at the cost of slightly more conservative,
    // possibly-slower charges). Exists because the formula's margin is
    // derived from characterized/observed conditions and can still run a
    // little tight on a setup that differs from those conditions; this is
    // the user's own manual lever for "still seeing overthrows, make it
    // more cautious right now" without waiting on more logged throws.
    float safety_margin_scale = 1.0f + (cfg->safety_margin_pct / 100.0f);
    coarse_stop *= safety_margin_scale;
    coarse_stop = fmaxf(0.10f, fminf(5.0f, coarse_stop));
    fine_stop *= safety_margin_scale;
    fine_stop = fmaxf(0.010f, fminf(0.50f, fine_stop));

    // --- Kp cap: commanded speed at the stop threshold must not exceed the
    // speed the tail was characterized at, or the real tail will be bigger
    // than what the threshold above assumed. This is the direct fix for
    // "suggested Kp overthrows" - it is not a tuning preference, it is
    // keeping the gain consistent with the threshold it will run against.
    //
    // The PID loop floors its commanded speed at the profile's own
    // min_flow_speed_rps (see charge_mode.cpp: fine_speed/coarse_speed is
    // clamped up to that floor even when Kp*error would ask for less). So
    // the speed actually running right at the stop threshold is not
    // Kp*threshold - it is max(Kp*threshold, currently configured min
    // flow speed). If someone applies only the suggested Kp/threshold and
    // leaves an old, higher min-speed floor in place (the "Apply" button
    // per row allows exactly that), the floor - not Kp - decides the real
    // speed at cutoff, and it can be well above what the threshold assumed,
    // which is what actually produces the overthrow. Folding the *current*
    // min flow speed into the reference speed here means the suggested
    // Kp/threshold pair stays safe even if the min-speed suggestion is
    // never applied.
    profile_t *existing_profile = profile_get_by_idx((uint8_t)profile_idx);
    float coarse_existing_min_speed = existing_profile ? existing_profile->coarse_min_flow_speed_rps : 0.0f;
    float fine_existing_min_speed = existing_profile ? existing_profile->fine_min_flow_speed_rps : 0.0f;

    float coarse_tail_ref_speed = model.coarse_trim_speed_rps > 0.01f
        ? model.coarse_trim_speed_rps : 0.3f;
    float fine_tail_ref_speed = model.fine_recovery_speed_rps > 0.001f
        ? model.fine_recovery_speed_rps : 0.08f;

    // If the tube's currently configured floor speed is higher than the
    // speed the tail was actually characterized at, the real in-flight tail
    // at cutoff will be bigger too (more material already committed at a
    // faster feed), so widen the stop threshold in proportion before
    // capping Kp against it - rather than capping Kp against a reference
    // speed the motor will never actually be allowed to slow below.
    if (coarse_existing_min_speed > coarse_tail_ref_speed) {
        coarse_stop *= (coarse_existing_min_speed / coarse_tail_ref_speed);
        coarse_stop = fmaxf(0.10f, fminf(5.0f, coarse_stop));
        coarse_tail_ref_speed = coarse_existing_min_speed;
    }
    if (fine_existing_min_speed > fine_tail_ref_speed) {
        fine_stop *= (fine_existing_min_speed / fine_tail_ref_speed);
        fine_stop = fmaxf(0.010f, fminf(0.50f, fine_stop));
        fine_tail_ref_speed = fine_existing_min_speed;
    }

    float kp_coarse_cap = coarse_tail_ref_speed / fmaxf(coarse_stop, 0.05f);
    float kp_fine_cap = fine_tail_ref_speed / fmaxf(fine_stop, 0.01f);
    kp_coarse = fminf(kp_coarse, kp_coarse_cap);
    kp_fine = fminf(kp_fine, kp_fine_cap);

    // --- live over-rate: real overthrows shrink Kp further and are the
    // reason the suggestion actually comes down with more logged charges,
    // rather than being stuck at whatever characterization said once.
    bool over_rate_applied = false;
    if (runtime_trustworthy && runtime_stats.over_rate > 0.15f) {
        // 15% over-rate -> ~15% shrink, capped at 40% shrink so one bad
        // batch cannot collapse the gain to near zero.
        float shrink = fmaxf(0.60f, 1.0f - runtime_stats.over_rate);
        kp_coarse *= shrink;
        kp_fine *= shrink;
        over_rate_applied = true;
    }

    float coarse_min = model.coarse_trim_speed_rps > 0.01f ? model.coarse_trim_speed_rps : 0.3f;
    float coarse_max = model.coarse_best_speed_rps > 0.01f ? model.coarse_best_speed_rps : 8.0f;
    float fine_min = model.fine_recovery_speed_rps > 0.001f ? model.fine_recovery_speed_rps : 0.08f;
    float fine_max = model.fine_best_speed_rps > 0.001f ? model.fine_best_speed_rps : 2.0f;

    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"success\":true,\"available\":true,\"profile_idx\":%d,"
        "\"measured\":{"
          "\"k_coarse_gps_per_rps\":%.4f,\"k_fine_gps_per_rps\":%.5f,"
          "\"coarse_tail_gn\":%.4f,\"fine_tail_gn\":%.4f,"
          "\"scale_period_ms\":%.1f,\"kernel_weight_gn\":%.4f,"
          "\"tau_coarse_s\":%.2f,\"tau_fine_s\":%.2f,"
          "\"kp_coarse_cap\":%.4f,\"kp_fine_cap\":%.4f,"
          "\"used_live_coarse_tail\":%s,\"used_live_fine_tail\":%s,"
          "\"over_rate_applied\":%s,\"live_observation_count\":%u,\"live_over_rate\":%.3f,"
          "\"safety_margin_pct\":%.1f},"
        "\"suggested\":{"
          "\"p3\":%.4f,\"p4\":0,\"p5\":0,"
          "\"p6\":%.3f,\"p7\":%.3f,"
          "\"p8\":%.4f,\"p9\":0,\"p10\":0,"
          "\"p11\":%.3f,\"p12\":%.3f,"
          "\"c5\":%.4f,\"c6\":%.4f,\"c26\":%.4f}}",
        http_json_header, profile_idx,
        k_coarse, k_fine, coarse_tail, fine_tail,
        scale_period_s * 1000.0f, kernel, tau_coarse, tau_fine,
        kp_coarse_cap, kp_fine_cap,
        used_live_coarse_tail ? "true" : "false",
        used_live_fine_tail ? "true" : "false",
        over_rate_applied ? "true" : "false",
        (unsigned)(have_runtime ? runtime_stats.observation_count : 0),
        have_runtime ? runtime_stats.over_rate : 0.0f,
        cfg->safety_margin_pct,
        kp_coarse, coarse_min, coarse_max,
        kp_fine, fine_min, fine_max,
        coarse_stop, fine_stop, accept_tol);
    return finalize_json_response(file, len);
}

bool http_rest_ai_kernel_weight(struct fs_file *file, int num_params,
                                char *params[], char *values[]) {    int profile_idx = (int)profile_get_selected_idx();
    bool has_weight = false;
    float weight_gn = 0.0f;

    for (int idx = 0; idx < num_params; idx++) {
        if (strcmp(params[idx], "profile_idx") == 0) {
            profile_idx = atoi(values[idx]);
        }
        else if (strcmp(params[idx], "weight_gn") == 0) {
            weight_gn = strtof(values[idx], NULL);
            has_weight = true;
        }
    }

    if (profile_idx < 0 || profile_idx >= MAX_PROFILE_CNT) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Invalid profile_idx\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    // has_weight distinguishes "clear the override" (weight_gn=0 explicitly
    // passed) from "just read the current value" (param omitted entirely).
    if (has_weight) {
        if (!ai_tuning_set_user_kernel_weight_gn((uint8_t)profile_idx, weight_gn)) {
            int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
                "%s{\"success\":false,\"error\":\"Invalid kernel weight\"}",
                http_json_header);
            return finalize_json_response(file, len);
        }
    }

    ai_profile_model_t model;
    memset(&model, 0, sizeof(model));
    ai_tuning_get_profile_model_copy((uint8_t)profile_idx, &model);

    float confidence = 0.0f;
    float effective = ai_tuning_effective_kernel_weight_gn(&model, &confidence);
    const char* source = model.user_kernel_weight_gn > 0.0f ? "user" :
                         (model.estimated_kernel_weight_gn > 0.0f ? "ai_estimate" : "none");

    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"success\":true,\"profile_idx\":%d,"
        "\"user_kernel_weight_gn\":%.4f,\"estimated_kernel_weight_gn\":%.4f,"
        "\"kernel_weight_confidence\":%.4f,\"effective_kernel_weight_gn\":%.4f,"
        "\"effective_confidence\":%.4f,\"source\":\"%s\"}",
        http_json_header,
        profile_idx,
        model.user_kernel_weight_gn,
        model.estimated_kernel_weight_gn,
        model.kernel_weight_confidence,
        effective,
        confidence,
        source);
    return finalize_json_response(file, len);
}

bool rest_ai_tuning_init(void) {
    rest_register_handler("/rest/ai_tuning_start", http_rest_ai_tuning_start);
    rest_register_handler("/rest/ai_machine_calibration_start", http_rest_ai_machine_calibration_start);
    rest_register_handler("/rest/ai_tuning_status", http_rest_ai_tuning_status);
    rest_register_handler("/rest/ai_tuning_apply", http_rest_ai_tuning_apply);
    rest_register_handler("/rest/ai_tuning_cancel", http_rest_ai_tuning_cancel);
    rest_register_handler("/rest/ai_tuning_history", http_rest_ai_tuning_history);
    rest_register_handler("/rest/ai_tuning_apply_refined", http_rest_ai_tuning_apply_refined);
    rest_register_handler("/rest/ai_tuning_clear_history", http_rest_ai_tuning_clear_history);
    rest_register_handler("/rest/ai_tuning_config", http_rest_ai_tuning_config_get);
    rest_register_handler("/rest/ai_tuning_config_set", http_rest_ai_tuning_config_set);
    rest_register_handler("/rest/ai_kernel_weight", http_rest_ai_kernel_weight);
    rest_register_handler("/rest/ai_suggestions", http_rest_ai_suggestions);

    printf("AI Tuning REST endpoints registered\n");
    return true;
}

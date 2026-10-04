#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "charge_timeline.h"
#include "charge_mode.h"
#include "scale.h"
#include "common.h"

typedef struct {
    uint32_t t_ms;          // since the start of the recording
    float weight;           // scale reading (NaN if invalid)
    int16_t coarse_crps;    // coarse motor command, 1/100 rps
    int16_t fine_crps;      // fine motor command, 1/100 rps
    uint8_t phase;          // charge_timeline_phase_t
} charge_timeline_sample_t;

typedef enum {
    TIMELINE_STATE_IDLE = 0,
    TIMELINE_STATE_RECORDING,
    TIMELINE_STATE_DONE,
    TIMELINE_STATE_ABORTED,
} timeline_state_t;

static const char *timeline_state_names[] = {"idle", "rec", "done", "abort"};

static struct {
    timeline_state_t state;
    uint32_t id;                // increments with every new recording
    uint32_t gen;               // increments whenever existing samples are rewritten (decimation / restart)
    float target;
    float tolerance;
    bool final_valid;
    float final_weight;
    uint32_t stride;            // sample every stride * BASE_PERIOD
    uint32_t tick_count;        // base periods since start
    bool motor_has_run;
    TickType_t start_tick;
    uint16_t count;
    charge_timeline_sample_t samples[CHARGE_TIMELINE_MAX_SAMPLES];
} tl;

static SemaphoreHandle_t tl_mutex = NULL;
static TaskHandle_t tl_task = NULL;


static int16_t rps_to_crps(float rps) {
    if (!isfinite(rps)) {
        return 0;
    }
    float v = rps * 100.0f;
    if (v > 32767.0f) v = 32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t) lroundf(v);
}


// Halve the buffer: keep every other sample, double the interval.
static void timeline_decimate_locked(void) {
    uint16_t n = 0;
    for (uint16_t i = 0; i < tl.count; i += 2) {
        tl.samples[n++] = tl.samples[i];
    }
    tl.count = n;
    tl.stride *= 2;
    tl.gen++;
}


static void timeline_take_sample_locked(void) {
    float coarse = 0.0f, fine = 0.0f;
    charge_mode_get_live_motor_commands(&coarse, &fine);

    uint8_t phase;
    if (coarse > 0.0f) {
        phase = CHARGE_TIMELINE_PHASE_COARSE;
        tl.motor_has_run = true;
    }
    else if (fine > 0.0f) {
        phase = CHARGE_TIMELINE_PHASE_FINE;
        tl.motor_has_run = true;
    }
    else {
        phase = tl.motor_has_run ? CHARGE_TIMELINE_PHASE_SETTLE : CHARGE_TIMELINE_PHASE_IDLE;
    }

    if (tl.count >= CHARGE_TIMELINE_MAX_SAMPLES) {
        timeline_decimate_locked();
    }

    charge_timeline_sample_t *s = &tl.samples[tl.count++];
    s->t_ms = (uint32_t) ((xTaskGetTickCount() - tl.start_tick) * portTICK_PERIOD_MS);
    s->weight = scale_get_current_measurement();
    s->coarse_crps = rps_to_crps(coarse);
    s->fine_crps = rps_to_crps(fine);
    s->phase = phase;
}


static void charge_timeline_task(void *p) {
    (void) p;
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CHARGE_TIMELINE_BASE_PERIOD_MS));

        if (xSemaphoreTake(tl_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
            continue;
        }
        if (tl.state == TIMELINE_STATE_RECORDING) {
            if (tl.tick_count % tl.stride == 0) {
                timeline_take_sample_locked();
            }
            tl.tick_count++;
        }
        xSemaphoreGive(tl_mutex);
    }
}


void charge_timeline_init(void) {
    if (tl_mutex != NULL) {
        return;
    }
    memset(&tl, 0, sizeof(tl));
    tl.stride = 1;
    tl_mutex = xSemaphoreCreateMutex();
    // Low priority: sampling is best-effort and must never delay charging.
    xTaskCreate(charge_timeline_task, "Charge Timeline", 512, NULL, 2, &tl_task);
}


void charge_timeline_begin(float target_weight, float tolerance) {
    if (tl_mutex == NULL) {
        charge_timeline_init();
    }
    xSemaphoreTake(tl_mutex, portMAX_DELAY);
    tl.id++;
    tl.gen++;
    tl.state = TIMELINE_STATE_RECORDING;
    tl.target = target_weight;
    tl.tolerance = tolerance;
    tl.final_valid = false;
    tl.final_weight = 0.0f;
    tl.stride = 1;
    tl.tick_count = 0;
    tl.motor_has_run = false;
    tl.count = 0;
    tl.start_tick = xTaskGetTickCount();
    timeline_take_sample_locked();      // t = 0
    xSemaphoreGive(tl_mutex);
}


void charge_timeline_finish(bool final_valid, float final_weight) {
    if (tl_mutex == NULL) {
        return;
    }
    xSemaphoreTake(tl_mutex, portMAX_DELAY);
    if (tl.state == TIMELINE_STATE_RECORDING) {
        timeline_take_sample_locked();  // closing sample
        tl.final_valid = final_valid && isfinite(final_weight);
        tl.final_weight = final_weight;
        tl.state = TIMELINE_STATE_DONE;
    }
    xSemaphoreGive(tl_mutex);
}


void charge_timeline_abort(void) {
    if (tl_mutex == NULL) {
        return;
    }
    xSemaphoreTake(tl_mutex, portMAX_DELAY);
    if (tl.state == TIMELINE_STATE_RECORDING) {
        tl.state = TIMELINE_STATE_ABORTED;
    }
    xSemaphoreGive(tl_mutex);
}


// GET /rest/charge_timeline?id=<id>&gen=<gen>&from=<index>
// Returns samples starting at `from` when id/gen match what the client already
// has, otherwise all samples (from = 0). Each sample: [t_ms, weight, coarse_rps,
// fine_rps, phase].
bool http_rest_charge_timeline(struct fs_file *file, int num_params, char *params[], char *values[]) {
    static char buf[CHARGE_TIMELINE_MAX_SAMPLES * 30 + 512];

    uint32_t client_id = 0, client_gen = 0;
    int from = 0;
    for (int i = 0; i < num_params; i++) {
        if (strcmp(params[i], "id") == 0) {
            client_id = (uint32_t) strtoul(values[i], NULL, 10);
        }
        else if (strcmp(params[i], "gen") == 0) {
            client_gen = (uint32_t) strtoul(values[i], NULL, 10);
        }
        else if (strcmp(params[i], "from") == 0) {
            from = atoi(values[i]);
        }
    }

    if (tl_mutex == NULL) {
        charge_timeline_init();
    }
    xSemaphoreTake(tl_mutex, portMAX_DELAY);

    if (client_id != tl.id || client_gen != tl.gen || from < 0 || from > tl.count) {
        from = 0;
    }

    size_t cap = sizeof(buf);
    int len = snprintf(buf, cap,
        "%s{\"id\":%lu,\"gen\":%lu,\"st\":\"%s\",\"tg\":%.3f,\"tol\":%.3f,"
        "\"fv\":%s,\"fw\":%.3f,\"dt\":%lu,\"n\":%u,\"from\":%d,\"d\":[",
        http_json_header,
        (unsigned long) tl.id, (unsigned long) tl.gen,
        timeline_state_names[tl.state],
        tl.target, tl.tolerance,
        boolean_to_string(tl.final_valid), tl.final_valid ? tl.final_weight : 0.0f,
        (unsigned long) (tl.stride * CHARGE_TIMELINE_BASE_PERIOD_MS),
        (unsigned int) tl.count, from);

    for (int i = from; i < tl.count && len > 0 && (size_t) len < cap - 64; i++) {
        const charge_timeline_sample_t *s = &tl.samples[i];
        float w = isfinite(s->weight) ? s->weight : 0.0f;
        len += snprintf(buf + len, cap - len, "%s[%lu,%.3f,%.2f,%.2f,%u]",
                        i > from ? "," : "",
                        (unsigned long) s->t_ms, w,
                        s->coarse_crps / 100.0f, s->fine_crps / 100.0f,
                        (unsigned int) s->phase);
    }
    xSemaphoreGive(tl_mutex);

    if (len > 0 && (size_t) len < cap - 3) {
        len += snprintf(buf + len, cap - len, "]}");
    }

    size_t data_length = strlen(buf);
    file->data = buf;
    file->len = data_length;
    file->index = data_length;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return true;
}

#include <FreeRTOS.h>
#include <queue.h>
#include <stdlib.h>
#include <semphr.h>
#include <time.h>
#include <math.h>
#include <task.h>
#include <stdbool.h>
#include <string.h>

#include "hardware/uart.h"
#include "configuration.h"
#include "scale.h"
#include "app.h"

const static char CMD_REQUEST_DATA_TRANSFER[] = "!p\r\n";
const static char CMD_CALIBRATE_FUNC[] = "!q\r\n";
// const static char CMD_COUNTING_FUNC[] = "!r\r\n";
const static char CMD_CHANGE_WEIGHT_UNIT[] = "!s\r\n";
const static char CMD_TARE_FUNC[] = "!t\r\n";
const static char CMD_BACKLIGHT[] = "!u\r\n";



typedef union {
    struct __attribute__((__packed__)){
        char header[2];
        char data[7];
        char unit[3];
        char terminator[2];
    };
    char bytes[14];
} gngscale_standard_data_format_t;


// ---------------------------------------------------------------------------
// Tare handling
// ---------------------------------------------------------------------------
// The listener task polls the scale continuously ("!p", wait for the 14 byte
// reply, 50 ms pause, repeat), so the scale is busy answering most of the
// time. Writing "!t" from another task at an arbitrary moment (as before)
// could land while the scale was still processing "!p" or transmitting its
// reply, and the JJ-B gives no acknowledgement, so a dropped tare was never
// noticed. Now force_zero() only raises a request; the listener task -- the
// only task that talks to the scale -- sends the tare between two polls once
// the line has been quiet, pauses polling while the scale tares, then checks
// that the reading came back to ~0 and repeats the tare if it did not.
#define GNG_TARE_QUIET_MS           40      // line must be idle this long before "!t"
#define GNG_TARE_SETTLE_MS          600     // no polling while the scale tares
#define GNG_TARE_VERIFY_SAMPLES     3       // valid readings used to check the result
#define GNG_TARE_VERIFY_WINDOW_MS   1500    // give up checking after this long
#define GNG_TARE_ZERO_BAND          0.05f   // |reading| that counts as "tared" (~3 digits at 0.001 g)
#define GNG_TARE_MAX_ATTEMPTS       3

static volatile bool tare_requested = false;

typedef struct {
    bool active;
    uint8_t attempts;
    uint8_t samples;
    float sum;
    TickType_t start_tick;
} gng_tare_verify_t;

static gng_tare_verify_t tare_verify = {0};

// Forward declaration
void _gng_scale_listener_task(void *p);
void scalegng_press_print_key();
void scalegng_press_tare_key();

extern scale_config_t scale_config;

// Instance of the scale handle for G&G JJB series
scale_handle_t gng_scale_handle = {
    .read_loop_task = _gng_scale_listener_task,
    .force_zero = scalegng_press_tare_key,
};

static float _decode_measurement_msg(gngscale_standard_data_format_t * msg) {
    // Decode header
    // Determine sign
    int sign = 1;

    // Check if the header contains a negative sign
    if (msg->header[0] == '-') {
        sign = -1;
    }

    // Decode weight information
    char *endptr;
    float weight = strtof(msg->data, &endptr);

    if( endptr == msg->data ) {
        // Conversion failed
        return nanf(msg->data);
    }

    // Apply the sign
    weight *= sign;

    return weight;
}

// Wait until nothing has arrived on the scale UART for GNG_TARE_QUIET_MS
// (bounded, so a scale that streams forever cannot stall the task).
static void _gng_wait_line_quiet(void) {
    TickType_t start = xTaskGetTickCount();
    TickType_t last_rx = start;
    while ((xTaskGetTickCount() - last_rx) < pdMS_TO_TICKS(GNG_TARE_QUIET_MS) &&
           (xTaskGetTickCount() - start) < pdMS_TO_TICKS(500)) {
        while (uart_is_readable(SCALE_UART)) {
            uart_getc(SCALE_UART);
            last_rx = xTaskGetTickCount();
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// Send the tare command from the listener task (line idle), then leave the
// scale alone while it tares.
static void _gng_send_tare(bool new_request) {
    _gng_wait_line_quiet();
    scale_write(CMD_TARE_FUNC, strlen(CMD_TARE_FUNC));
    vTaskDelay(pdMS_TO_TICKS(GNG_TARE_SETTLE_MS));

    if (new_request) {
        tare_verify.attempts = 0;
    }
    tare_verify.attempts++;
    tare_verify.active = true;
    tare_verify.samples = 0;
    tare_verify.sum = 0.0f;
    tare_verify.start_tick = xTaskGetTickCount();
}

// Feed each decoded reading; returns true when the tare has to be repeated.
static bool _gng_verify_tare(float reading) {
    if (!tare_verify.active) {
        return false;
    }
    if ((xTaskGetTickCount() - tare_verify.start_tick) > pdMS_TO_TICKS(GNG_TARE_VERIFY_WINDOW_MS)) {
        // Could not get enough valid readings (e.g. scale still showing dashes)
        tare_verify.active = false;
        return tare_verify.attempts < GNG_TARE_MAX_ATTEMPTS;
    }
    if (!isfinite(reading)) {
        return false;
    }
    tare_verify.sum += reading;
    tare_verify.samples++;
    if (tare_verify.samples < GNG_TARE_VERIFY_SAMPLES) {
        return false;
    }
    tare_verify.active = false;
    float mean = tare_verify.sum / tare_verify.samples;
    return fabsf(mean) > GNG_TARE_ZERO_BAND && tare_verify.attempts < GNG_TARE_MAX_ATTEMPTS;
}

//read UART
void _gng_scale_listener_task(void *p) {
    uint8_t string_buf_idx = 0;
    gngscale_standard_data_format_t frame;
    bool repeat_tare = false;

    while (true) {
        // Tare requested (or the last one did not take): send it now, between
        // two polls, while the line is idle.
        if (tare_requested) {
            tare_requested = false;
            repeat_tare = false;
            _gng_send_tare(true);
        }
        else if (repeat_tare) {
            repeat_tare = false;
            _gng_send_tare(false);
        }

        // Reset frame buffer index at the start of each poll cycle (fixes partial-frame carryover on timeout)
        string_buf_idx = 0;

        // Clear any stale data in RX buffer before sending request
        while (uart_is_readable(SCALE_UART)) {
            uart_getc(SCALE_UART);
        }

        // Request for a data transfer using mutex-protected write
        scale_write(CMD_REQUEST_DATA_TRANSFER, strlen(CMD_REQUEST_DATA_TRANSFER));

        // Wait for scale to respond (up to 200ms)
        TickType_t start_tick = xTaskGetTickCount();
        bool got_response = false;

        while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(200)) {
            while (uart_is_readable(SCALE_UART)) {
                char ch = uart_getc(SCALE_UART);
                frame.bytes[string_buf_idx++] = ch;

                // If we have received 14 bytes then we can decode the message
                if (string_buf_idx == sizeof(gngscale_standard_data_format_t)) {
                    // Data is ready, send to decode
                    scale_config.current_scale_measurement = _decode_measurement_msg(&frame);
                    if (_gng_verify_tare(scale_config.current_scale_measurement)) {
                        repeat_tare = true;
                    }
                    // Signal the data is ready
                    if (scale_config.scale_measurement_ready) {
                        xSemaphoreGive(scale_config.scale_measurement_ready);
                    }

                    // Reset
                    string_buf_idx = 0;
                    got_response = true;
                }

                // \n is the terminator. Reset on receiving it.
                if (ch == '\n') {
                    string_buf_idx = 0;
                    if (got_response) {
                        break;
                    }
                }
            }

            if (got_response) {
                break;
            }

            // Small delay before checking again
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Wait before next request
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

//G&G JJB key function
//C4 communication setting - data signal command control 
//standard ESC 0x1B to ! 0x22
//hint:
//For better syntax change standard setting 'C4' 27 'ESC' symbol at scale to 33 '!' sign 

//ESC p -> 0x1b 0x70 0x0D 0x0A standard setting
// ! p -> 0x21 0x70 0x0D 0x0A
void scalegng_press_print_key() {
    scale_write(CMD_REQUEST_DATA_TRANSFER, strlen(CMD_REQUEST_DATA_TRANSFER));
}

//ESC t -> 0x1B 0x74 0x0D 0x0A standard setting
// ! t -> 0x21 0x74 0x0D 0x0A
void scalegng_press_tare_key() {
    // Handled by the listener task between two polls (see "Tare handling")
    tare_requested = true;
}

//ESC s -> 0x1B 0x73 0x0D 0x0A standard setting
// ! s -> 0x21 0x73 0x0D 0x0A
void scalegng_press_weight_key() {
    scale_write(CMD_CHANGE_WEIGHT_UNIT, strlen(CMD_CHANGE_WEIGHT_UNIT));
}

//ESC q -> 0x1B 0x71 0x0D 0x0A standard setting
// ! q -> 0x21 0x71 0x0D 0x0A
void scalegng_press_cal_key() {
    scale_write(CMD_CALIBRATE_FUNC, strlen(CMD_CALIBRATE_FUNC));
}

// ESC u -> 0x1B 0x75 0x0D 0x0A standard setting
// ! u -> 0x21 0x75 0x0D 0x0A
void scalegng_display_light() {
    scale_write(CMD_BACKLIGHT, strlen(CMD_BACKLIGHT));
}

// AppState_t scale_enable_fast_report(AppState_t prev_state) {
//     // TODO: Finish this
// }

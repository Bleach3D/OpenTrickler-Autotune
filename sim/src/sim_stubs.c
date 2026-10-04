// Hardware stubs for the OpenTrickler simulator.
//
// Replaces the real peripherals (scale UART, steppers, EEPROM, flash,
// display, LEDs, servo gate) with in-memory equivalents wired to the plant.
// All RTOS primitives, the clock and the plant ticker live in sim_rt.cpp;
// this file must not define any of them.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <u8g2.h>

#include "scale.h"
#include "motors.h"
#include "eeprom.h"
#include "neopixel_led.h"
#include "servo_gate.h"
#include "display.h"
#include "mini_12864_module.h"
#include "app_state.h"
#include "flash_storage.h"
#include "common.h"

#include "sim_plant.h"
#include "sim_harness.h"

extern sim_plant_config_t g_plant_cfg;
extern sim_plant_state_t g_plant_state;
double sim_plant_take_reading(const sim_plant_config_t *cfg, sim_plant_state_t *st, uint32_t dt_ms);

// ---------------------------------------------------------------------------
// Scale
// ---------------------------------------------------------------------------
scale_config_t scale_config;
static TickType_t g_last_scale_tick = 0;

float scale_get_current_measurement(void) {
    return (float)g_plant_state.reported_weight_gn;
}

bool scale_block_wait_for_next_measurement(uint32_t block_time_ms, float *current_measurement) {
    // Real hardware streams at a fixed rate; block until the next sample is
    // due so the control loop is paced exactly as it is on the device.
    TickType_t period = g_plant_cfg.scale_period_ms ? g_plant_cfg.scale_period_ms : 100;
    TickType_t start = xTaskGetTickCount();
    for (;;) {
        TickType_t now = xTaskGetTickCount();
        TickType_t since = now - g_last_scale_tick;
        if (since >= period) {
            g_last_scale_tick = now;
            double reading = sim_plant_take_reading(&g_plant_cfg, &g_plant_state, (uint32_t)since);
            if (current_measurement) *current_measurement = (float)reading;
            return true;
        }
        if ((now - start) >= block_time_ms) return false;
        vTaskDelay(1);
    }
}

void scale_press_re_zero_key(void) { sim_plant_zero(&g_plant_state); }
void scale_press_print_key(void) {}
void scale_press_sample_key(void) {}
void scale_press_mode_key(void) {}
void scale_press_cal_key(void) {}
void scale_press_on_off_key(void) {}
bool scale_init(void) { return true; }
bool scale_config_init(void) { return true; }
bool scale_config_save(void) { return true; }
void scale_listener_task(void *p) { (void)p; }

// ---------------------------------------------------------------------------
// Motors -> plant
// ---------------------------------------------------------------------------
void motor_set_speed(motor_select_t selected_motor, float new_velocity) {
    if (selected_motor == SELECT_COARSE_TRICKLER_MOTOR) {
        sim_plant_set_coarse_rps(&g_plant_state, new_velocity);
    } else if (selected_motor == SELECT_FINE_TRICKLER_MOTOR) {
        sim_plant_set_fine_rps(&g_plant_state, new_velocity);
    }
    sim_harness_on_motor_command(selected_motor, new_velocity);
}

uint16_t get_motor_max_speed(motor_select_t m) { return (m == SELECT_COARSE_TRICKLER_MOTOR) ? 12 : 8; }
float get_motor_min_speed(motor_select_t m) { (void)m; return 0.05f; }
void motor_enable(motor_select_t m, bool e) { (void)m; (void)e; }
bool motor_config_init(void) { return true; }
bool motor_config_save(void) { return true; }
void motor_task(void *p) { (void)p; }

// ---------------------------------------------------------------------------
// EEPROM / flash: plain RAM
// ---------------------------------------------------------------------------
#define SIM_EEPROM_SIZE (32 * 1024)
static uint8_t g_eeprom[SIM_EEPROM_SIZE];

bool eeprom_read(uint16_t addr, uint8_t *data, size_t len) {
    if ((size_t)addr + len > SIM_EEPROM_SIZE) return false;
    memcpy(data, &g_eeprom[addr], len);
    return true;
}
bool eeprom_write(uint16_t addr, uint8_t *data, size_t len) {
    if ((size_t)addr + len > SIM_EEPROM_SIZE) return false;
    memcpy(&g_eeprom[addr], data, len);
    return true;
}
void eeprom_register_handler(eeprom_save_handler_t h) { (void)h; }
bool eeprom_init(void) { return true; }
uint8_t eeprom_save_all(void) { return 0; }

#define SIM_ML_HISTORY_SIZE (64 * 1024)
static uint8_t g_ml_history[SIM_ML_HISTORY_SIZE];

bool flash_ml_history_read(uint8_t *data, size_t len) {
    if (len > SIM_ML_HISTORY_SIZE) return false;
    memcpy(data, g_ml_history, len);
    return true;
}
bool flash_ml_history_write(const uint8_t *data, size_t len) {
    if (len > SIM_ML_HISTORY_SIZE) return false;
    memcpy(g_ml_history, data, len);
    return true;
}

// ---------------------------------------------------------------------------
// Display / LEDs / servo gate / buttons: inert
// ---------------------------------------------------------------------------
static u8g2_t g_display;
u8g2_t *get_display_handler(void) { return &g_display; }

const uint8_t u8g2_font_helvB08_tr[1] = {0};
const uint8_t u8g2_font_helvR08_tr[1] = {0};
const uint8_t u8g2_font_helvB10_tr[1] = {0};
const uint8_t u8g2_font_helvB12_tr[1] = {0};
const uint8_t u8g2_font_helvR10_tr[1] = {0};
const uint8_t u8g2_font_helvR12_tr[1] = {0};
const uint8_t u8g2_font_5x7_tr[1] = {0};
const uint8_t u8g2_font_6x13_tr[1] = {0};
const uint8_t u8g2_font_courB08_tr[1] = {0};
const uint8_t u8g2_font_courR08_tr[1] = {0};
const uint8_t u8g2_font_profont22_tf[1] = {0};

neopixel_led_config_t neopixel_led_config;
void neopixel_led_set_colour(rgbw_u32_t a, rgbw_u32_t b, rgbw_u32_t c, bool w) { (void)a;(void)b;(void)c;(void)w; }
void _neopixel_led_set_colour(uint32_t a, uint32_t b, uint32_t c) { (void)a;(void)b;(void)c; }

servo_gate_t servo_gate;
void servo_gate_set_ratio(float r, bool w) { (void)r; (void)w; }

AppState_t exit_state;
QueueHandle_t encoder_event_queue = NULL;

ButtonEncoderEvent_t button_wait_for_input(bool block) { (void)block; return BUTTON_NO_EVENT; }

// Normally in pid_autotune.cpp, which the simulator does not build.
bool pid_autotune_is_active(void) { return false; }

uint16_t swuart_calcCRC(uint8_t *data, size_t len) { (void)data; (void)len; return 0; }
void busy_wait_us(uint64_t us) { vTaskDelay((TickType_t)(us / 1000)); }

// Normally in neopixel_led.c, which the simulator does not build.
uint32_t hex_string_to_decimal(char *string) {
    if (string == NULL) return 0;
    if (*string == '#') string++;
    return (uint32_t)strtoul(string, NULL, 16);
}

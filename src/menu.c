#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stdio.h>
#include <u8g2.h>
#include <mui.h>
#include <mui_u8g2.h>

#include "app.h"
#include "configuration.h"
#include "scale.h"
#include "display.h"
#include "mini_12864_module.h"
#include "eeprom.h"
#include "charge_mode.h"
#include "cleanup_mode.h"
#include "pid_autotune.h"
#include "eeprom.h"
#include "wireless.h"
#include "system_control.h"
#include "app_state.h"
#include "autotune_menu.h"
#include "splash_screen.h"

// External variables
extern muif_t muif_list[];
extern fds_t fds_data[];
extern const size_t muif_cnt;

// External menus

// Local variables
extern QueueHandle_t encoder_event_queue;
extern charge_mode_config_t charge_mode_config;


// Reset button = "one page back" on the menu pages. Each form maps to the
// page it was reached from (mostly the same target as its Back / <-Return
// item). Returns 0 when there is nowhere to go back to (main menu), and
// 0xFF for the profile picker child list, which returns to whichever form
// opened it. The weight-entry page depends on the profile's decimal places.
#define MENU_BACK_NONE      0
#define MENU_BACK_RESTORE   0xFF

static uint8_t menu_weight_form(uint8_t form_2dp) {
    return (charge_mode_config.eeprom_charge_mode_data.decimal_places == DP_3) ? form_2dp + 1 : form_2dp;
}

static uint8_t menu_back_target(int form_id) {
    switch (form_id) {
        // Start / charge
        case 10: return 1;
        case 11: case 12: return 10;
        case 13: return menu_weight_form(11);

        // Autotune
        case 70: return 1;
        case 71: return 70;
        case 72: return 70;
        case 73: return 72;
        case 74: case 75: return 73;
        case 76: return 72;
        case 77: case 78: return 76;
        case 79: return menu_weight_form(77);
        case 80: return menu_weight_form(74);

        // Cleanup / wireless / settings
        case 20: return 1;
        case 40: return 1;
        case 41: return 40;
        case 30: return 1;
        case 31: return 30;
        case 32: return 30;
        case 33: return MENU_BACK_RESTORE;     // profile picker child list
        case 34: return 32;
        case 38: return 34;
        case 35: case 36: case 37: case 39: return 30;
        case 51: case 53: return 31;
        case 60: case 61: return 37;

        case 1:
        default:
            return MENU_BACK_NONE;
    }
}


static void menu_go_back(mui_t *mui) {
    // While a number / option field is being edited, Reset first just ends
    // the edit (value kept), like pushing the encoder would.
    if (mui->is_mud) {
        mui_SendSelect(mui);
        return;
    }

    uint8_t target = menu_back_target(mui_GetCurrentFormId(mui));
    if (target == MENU_BACK_RESTORE) {
        mui_RestoreForm(mui);
    }
    else if (target != MENU_BACK_NONE) {
        mui_GotoFormAutoCursorPosition(mui, target);
    }
}

#define SPLASH_FRAME_PERIOD_MS  40      // ~25 fps


// Animated boot splash (~10 s). Pushing the encoder or the reset button skips
// it. Runs inside the menu task, i.e. after the scheduler has started, so the
// WiFi task connects in the background meanwhile.
static void run_boot_splash(u8g2_t *display_handler) {
    TickType_t start = xTaskGetTickCount();
    TickType_t last_wake = start;

    while (true) {
        uint32_t t_ms = (uint32_t) ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS);
        if (t_ms >= SPLASH_SCREEN_DURATION_MS) {
            break;
        }

        u8g2_ClearBuffer(display_handler);
        splash_screen_draw_frame(display_handler, t_ms);
        u8g2_SendBuffer(display_handler);

        // Handle all input collected since the last frame
        bool skip = false;
        ButtonEncoderEvent_t ev;
        while ((ev = button_wait_for_input(false)) != BUTTON_NO_EVENT) {
            if (ev == BUTTON_ENCODER_PRESSED || ev == BUTTON_RST_PRESSED) {
                skip = true;
            }
            else if (ev == OVERRIDE_FROM_REST) {
                // Something was started from the web UI: stop the splash and
                // hand the event back to the menu loop untouched.
                if (encoder_event_queue != NULL) {
                    xQueueSendToFront(encoder_event_queue, &ev, 0);
                }
                skip = true;
                break;
            }
            // Encoder rotation is ignored here
        }
        if (skip) {
            break;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SPLASH_FRAME_PERIOD_MS));
    }

    u8g2_ClearBuffer(display_handler);
    u8g2_SendBuffer(display_handler);
}


void menu_task(void *p){
    u8g2_t * display_handler = get_display_handler();
    // Create UI element
    mui_t mui;

    run_boot_splash(display_handler);

    mui_Init(&mui, display_handler, fds_data, muif_list, muif_cnt);
    mui_GotoForm(&mui, 1, 0);

    // Render the menu before user input
    u8g2_ClearBuffer(display_handler);
    mui_Draw(&mui);
    u8g2_SendBuffer(display_handler);

    while (true) {
        if (mui_IsFormActive(&mui)) {
            // Block wait for the user input
            ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(true);
            if (button_encoder_event == BUTTON_ENCODER_ROTATE_CW) {
                mui_NextField(&mui);
            }
            else if (button_encoder_event == BUTTON_ENCODER_ROTATE_CCW) {
                mui_PrevField(&mui);
            }
            else if (button_encoder_event == BUTTON_ENCODER_PRESSED) {
                mui_SendSelect(&mui);
            }
            else if (button_encoder_event == BUTTON_RST_PRESSED) {
                menu_go_back(&mui);
            }
            else if (button_encoder_event == OVERRIDE_FROM_REST) {
                // Assuming the caller code will set the exit_state
                mui_SaveForm(&mui);          // store the current form and position so that the child can jump back
                mui_LeaveForm(&mui);
            }
        }
        else {
            uint8_t exit_form_id = 1;  // by default it goes to the main menu
            // menu is not active, leave the control to the app
            switch (exit_state) {
                case APP_STATE_ENTER_CHARGE_MODE:
                    exit_form_id = charge_mode_menu(false);
                    break;
                case APP_STATE_ENTER_CHARGE_MODE_FROM_REST:
                    exit_form_id = charge_mode_menu(true);
                    break;
                case APP_STATE_ENTER_CLEANUP_MODE:
                    exit_form_id = cleanup_mode_menu();
                    break;
                case APP_STATE_ENTER_SCALE_CALIBRATION:
                    exit_form_id = scale_calibrate_with_external_weight();
                    break;
                case APP_STATE_ENTER_EEPROM_SAVE: 
                    exit_form_id = eeprom_save_all();
                    break;
                case APP_STATE_ENTER_EEPROM_ERASE:
                    exit_form_id = eeprom_erase(true);
                    break;
                case APP_STATE_ENTER_REBOOT:
                    exit_form_id = software_reboot();
                    break;
                case APP_STATE_ENTER_WIFI_INFO:
                    exit_form_id = wireless_view_wifi_info();
                    break;
                case APP_STATE_ENTER_PID_AUTOTUNE_FROM_REST:
                    exit_form_id = pid_autotune_menu();
                    break;
                case APP_STATE_ENTER_PID_AUTOTUNE:
                    exit_form_id = autotune_pid_menu();
                    break;
                case APP_STATE_ENTER_AI_TUNING:
                    exit_form_id = autotune_ai_menu(false);
                    break;
                case APP_STATE_ENTER_AI_MACHINE_CAL:
                    exit_form_id = autotune_ai_menu(true);
                    break;
                default:
                    break;
            }

            exit_state = APP_STATE_DEFAULT;
            mui_GotoForm(&mui, exit_form_id, 0);
        }

        u8g2_ClearBuffer(display_handler);
        mui_Draw(&mui);
        u8g2_SendBuffer(display_handler);
    }
}

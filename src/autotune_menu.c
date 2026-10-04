// Encoder-menu entry points for PID Autotune and AI Tuning.
//
// Before this, both tuning modes could only be started from the web GUI
// (REST). These wrappers start the same routines from the physical menu and
// add the on-device "save / discard" step the web GUI normally provides:
//
//   PID Autotune : pid_autotune_menu() fits and applies the values to the
//                  profile in RAM; afterwards we offer to save to EEPROM.
//   AI Tuning    : ai_tuning_start() / ai_tuning_start_machine_calibration()
//                  then the normal charge mode runs the characterization
//                  charges; afterwards we offer to save or discard the model.
//
// Menu forms: see mui_menu.c (forms 70-80).

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stdio.h>
#include <string.h>
#include <u8g2.h>

#include "app.h"
#include "autotune_menu.h"
#include "display.h"
#include "mini_12864_module.h"
#include "charge_mode.h"
#include "profile.h"
#include "pid_autotune.h"
#include "ai_tuning.h"

extern charge_mode_config_t charge_mode_config;

#define AUTOTUNE_MENU_FORM_MAIN     1
#define AUTOTUNE_MENU_FORM_AI       72


// Throw away any pending encoder/button events (e.g. the RST event that
// ai_tuning_cancel() injects to kick charge mode out of its loop).
static void drain_input_events(void) {
    while (button_wait_for_input(false) != BUTTON_NO_EVENT) {
    }
}


// Simple full-screen message: bold title, rule, up to three lines and a hint
// line at the bottom. Blocks until the encoder is pressed or RST is pressed
// and returns which one it was.
static ButtonEncoderEvent_t show_message(const char *title,
                                         const char *line1,
                                         const char *line2,
                                         const char *line3,
                                         const char *hint) {
    u8g2_t *u8g2 = get_display_handler();

    u8g2_ClearBuffer(u8g2);
    u8g2_SetDrawColor(u8g2, 1);

    u8g2_SetFont(u8g2, u8g2_font_helvB08_tr);
    u8g2_DrawStr(u8g2, 5, 10, title);
    u8g2_DrawHLine(u8g2, 0, 13, u8g2_GetDisplayWidth(u8g2));

    u8g2_SetFont(u8g2, u8g2_font_helvR08_tr);
    if (line1) u8g2_DrawStr(u8g2, 5, 24, line1);
    if (line2) u8g2_DrawStr(u8g2, 5, 34, line2);
    if (line3) u8g2_DrawStr(u8g2, 5, 44, line3);
    if (hint)  u8g2_DrawStr(u8g2, 5, 61, hint);

    u8g2_SendBuffer(u8g2);

    while (true) {
        ButtonEncoderEvent_t ev = button_wait_for_input(true);
        if (ev == BUTTON_ENCODER_PRESSED || ev == BUTTON_RST_PRESSED) {
            return ev;
        }
    }
}


uint8_t autotune_pid_menu(void) {
    pid_autotune_menu();

    if (pid_autotune.result_valid && pid_autotune.applied_to_profile) {
        drain_input_events();
        ButtonEncoderEvent_t ev = show_message("PID Autotune done",
                                               "New values are active",
                                               "for this profile.",
                                               "Save to EEPROM?",
                                               "Push: Save  Reset: Skip");
        if (ev == BUTTON_ENCODER_PRESSED) {
            bool ok = profile_data_save();
            show_message(ok ? "Saved" : "Save failed",
                         ok ? "PID values stored." : "EEPROM write failed.",
                         NULL, NULL, "Push to continue");
        }
        else {
            show_message("Not saved",
                         "Values stay active",
                         "until reboot.",
                         NULL, "Push to continue");
        }
    }

    return AUTOTUNE_MENU_FORM_MAIN;
}


uint8_t autotune_ai_menu(bool machine_calibration) {
    const char *title = machine_calibration ? "Machine Calibration" : "AI Characterize";

    uint8_t profile_idx = (uint8_t) profile_get_selected_idx();
    profile_t *profile = profile_get_selected();
    if (profile == NULL) {
        show_message(title, "No profile selected.", NULL, NULL, "Push to continue");
        return AUTOTUNE_MENU_FORM_AI;
    }

    // The weight digits are interpreted with this profile's decimal places.
    charge_mode_data_load_for_profile(profile_idx);
    float target_weight = charge_mode_target_weight_from_digits();
    if (target_weight <= 0.0f) {
        show_message(title, "Charge weight must", "be above zero.", NULL, "Push to continue");
        return AUTOTUNE_MENU_FORM_AI;
    }

    bool started = machine_calibration
        ? ai_tuning_start_machine_calibration(profile, target_weight)
        : ai_tuning_start(profile, target_weight);

    if (!started) {
        if (machine_calibration) {
            show_message(title, "Needs a saved powder", "characterization for", "this profile first.", "Push to continue");
        }
        else {
            show_message(title, "Could not start", "AI tuning.", NULL, "Push to continue");
        }
        return AUTOTUNE_MENU_FORM_AI;
    }

    // Run the characterization / calibration charges through the normal
    // charge mode, exactly like the REST start does.
    charge_mode_menu(false);

    if (ai_tuning_is_complete()) {
        drain_input_events();
        ButtonEncoderEvent_t ev = show_message(machine_calibration ? "Calibration done" : "Characterize done",
                                               "Save the result to",
                                               "this profile's model?",
                                               NULL,
                                               "Push: Save  Reset: Drop");
        if (ev == BUTTON_ENCODER_PRESSED) {
            bool ok = ai_tuning_apply_params();
            show_message(ok ? "Saved" : "Save failed",
                         ok ? "AI model stored." : "No result to save.",
                         NULL, NULL, "Push to continue");
        }
        else {
            ai_tuning_cancel();
            drain_input_events();
            show_message("Dropped", "Result was not saved.", NULL, NULL, "Push to continue");
        }
    }
    else if (ai_tuning_is_active()) {
        // Charge mode was left with Reset before the run finished.
        ai_tuning_cancel();
        drain_input_events();
        show_message("Cancelled", "AI tuning stopped.", "Nothing was saved.", NULL, "Push to continue");
    }

    return AUTOTUNE_MENU_FORM_MAIN;
}

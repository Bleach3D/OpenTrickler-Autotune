#include <string.h>
#include <stdlib.h>
#include "rest_endpoints.h"
#include "http_rest.h"
#include "charge_mode.h"
#include "motors.h"
#include "scale.h"
#include "wireless.h"
#include "eeprom.h"
#include "mini_12864_module.h"
#include "display.h"
#include "neopixel_led.h"
#include "profile.h"
#include "cleanup_mode.h"
#include "servo_gate.h"
#include "system_control.h"
#include "rest_errors.h"
#include "rest_ai_tuning.h"
#include "ai_tuning.h"
#include "rest_pid_autotune.h"
#include "pid_autotune.h"
#include "flash_storage.h"
#include "display_config.h"
#include "ota_update.h"
#include "ota_client.h"

// Generated headers by html2header.py under scripts
#include "display_mirror.html.h"
#include "web_portal.html.h"
#include "web_portal_basic.html.h"
#include "wizard.html.h"
#include "styles.css.h"
#include "favicon.ico.h"
#include "apple_touch_icon.png.h"
#include "icon_192.png.h"
#include "icon_512.png.h"
#include "manifest.json.h"


bool http_404_error(struct fs_file *file, int num_params, char *params[], char *values[]) {

    file->data = "HTTP/1.1 404 Not Found\r\nContent-Type: application/json\r\n\r\n"
                 "{\"error\":404}";
    file->len = 13;
    file->index = 13;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}


bool http_display_mirror(struct fs_file *file, int num_params, char *params[], char *values[]) {
    size_t len = strlen(html_display_mirror_html);

    file->data = html_display_mirror_html;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT;

    return true;
}


bool http_web_portal(struct fs_file *file, int num_params, char *params[], char *values[]) {
    size_t len = strlen(html_web_portal_html);

    file->data = html_web_portal_html;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT;

    return true;
}


bool http_wizard(struct fs_file *file, int num_params, char *params[], char *values[]) {
    size_t len = strlen(html_wizard_html);

    file->data = html_wizard_html;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT;

    return true;
}


bool http_web_portal_basic(struct fs_file *file, int num_params, char *params[], char *values[]) {
    size_t len = strlen(html_web_portal_basic_html);

    file->data = html_web_portal_basic_html;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT;

    return true;
}


bool http_styles_css(struct fs_file *file, int num_params, char *params[], char *values[]) {
    size_t len = strlen(css_styles_css);

    file->data = css_styles_css;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT;

    return true;
}


bool http_favicon(struct fs_file *file, int num_params, char *params[], char *values[]) {
    file->data = (const char *)favicon_ico;
    file->len = favicon_ico_len;
    file->index = favicon_ico_len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT;

    return true;
}

// iOS Safari's "Add to Home Screen" ignores /favicon.ico entirely -- it only
// ever looks for a real PNG at an apple-touch-icon link (never a data: URI
// reliably, and never an .ico), which is why the home screen icon showed up
// blank before this existed. icon_192/icon_512 back a manifest.json so
// Android/Chrome's "Add to Home Screen" gets a proper icon too. All four use
// the original OpenTrickler favicon.
bool http_apple_touch_icon(struct fs_file *file, int num_params, char *params[], char *values[]) {
    file->data = (const char *)apple_touch_icon_png;
    file->len = apple_touch_icon_png_len;
    file->index = apple_touch_icon_png_len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT;

    return true;
}

bool http_icon_192(struct fs_file *file, int num_params, char *params[], char *values[]) {
    file->data = (const char *)icon_192_png;
    file->len = icon_192_png_len;
    file->index = icon_192_png_len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT;

    return true;
}

bool http_icon_512(struct fs_file *file, int num_params, char *params[], char *values[]) {
    file->data = (const char *)icon_512_png;
    file->len = icon_512_png_len;
    file->index = icon_512_png_len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT;

    return true;
}

bool http_manifest_json(struct fs_file *file, int num_params, char *params[], char *values[]) {
    file->data = (const char *)manifest_json;
    file->len = manifest_json_len;
    file->index = manifest_json_len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT;

    return true;
}


bool rest_endpoints_init(bool default_wizard) {
    if (default_wizard) {
        rest_register_handler("/", http_wizard);
    }
    else {
        rest_register_handler("/", http_web_portal);
    }
    
    rest_register_handler("/mobile", http_web_portal);
    rest_register_handler("/wizard", http_wizard);
    rest_register_handler("/404", http_404_error);
    rest_register_handler("/rest/scale_action", http_rest_scale_action);
    rest_register_handler("/rest/scale_config", http_rest_scale_config);
    rest_register_handler("/rest/charge_mode_config", http_rest_charge_mode_config);
    rest_register_handler("/rest/charge_mode_state", http_rest_charge_mode_state);
    rest_register_handler("/rest/cleanup_mode_state", http_rest_cleanup_mode_state);
    rest_register_handler("/rest/system_control", http_rest_system_control);
    rest_register_handler("/rest/coarse_motor_config", http_rest_coarse_motor_config);
    rest_register_handler("/rest/fine_motor_config", http_rest_fine_motor_config);
    rest_register_handler("/rest/button_control", http_rest_button_control);
    rest_register_handler("/rest/mini_12864_config", http_rest_mini_12864_module_config);
    rest_register_handler("/rest/wireless_config", http_rest_wireless_config);
    rest_register_handler("/rest/neopixel_led_config", http_rest_neopixel_led_config);
    rest_register_handler("/rest/profile_config", http_rest_profile_config);
    rest_register_handler("/rest/profile_summary", http_rest_profile_summary);
    rest_register_handler("/rest/servo_gate_state", http_rest_servo_gate_state);
    rest_register_handler("/rest/servo_gate_config", http_rest_servo_gate_config);
    rest_register_handler("/display_buffer", http_get_display_buffer);
    rest_register_handler("/display_mirror", http_display_mirror);

    // Additional web resources
    rest_register_handler("/basic", http_web_portal_basic);
    rest_register_handler("/styles.css", http_styles_css);
    rest_register_handler("/favicon.ico", http_favicon);
    rest_register_handler("/apple-touch-icon.png", http_apple_touch_icon);
    // iOS also probes this exact filename first, before falling back to the
    // plain apple-touch-icon.png link in <head> -- serve the same icon for
    // both so neither path shows a blank/default icon.
    rest_register_handler("/apple-touch-icon-precomposed.png", http_apple_touch_icon);
    rest_register_handler("/icon-192.png", http_icon_192);
    rest_register_handler("/icon-512.png", http_icon_512);
    rest_register_handler("/manifest.json", http_manifest_json);

    // Error reporting endpoints
    rest_register_handler("/rest/errors", http_rest_errors);
    rest_register_handler("/rest/clear_errors", http_rest_clear_errors);

    // Display configuration endpoint
    rest_register_handler("/rest/display_config", http_rest_display_config);

    // OTA firmware staging endpoints
    ota_update_init();
    rest_register_handler("/rest/ota_status", http_rest_ota_status);
    rest_register_handler("/rest/ota_begin", http_rest_ota_begin);
    rest_register_handler("/rest/ota_chunk", http_rest_ota_chunk);
    rest_register_handler("/rest/ota_finalize", http_rest_ota_finalize);
    rest_register_handler("/rest/ota_abort", http_rest_ota_abort);
    rest_register_handler("/rest/ota_apply", http_rest_ota_apply);

    ota_client_init();
    rest_register_handler("/rest/update_config", http_rest_update_config);
    rest_register_handler("/rest/update_check", http_rest_update_check);
    rest_register_handler("/rest/update_status", http_rest_update_status);
    rest_register_handler("/rest/update_apply", http_rest_update_apply);

    // Initialize flash storage for ML history
    flash_storage_init();

    // Initialize AI tuning system and REST endpoints
    ai_tuning_init();
    rest_ai_tuning_init();

    // Initialize PID autotune ("Learn Powder"-style characterisation) and REST endpoints
    pid_autotune_init();
    rest_pid_autotune_init();

    return true;
}

#ifndef APP_H_
#define APP_H_

#include "eeprom.h"

#define EEPROM_APP_CONFIG_DATA_REV              1           // 16 byte

typedef enum {
    APP_STATE_DEFAULT = 0,
    APP_STATE_ENTER_CHARGE_MODE = 1,
    APP_STATE_ENTER_CHARGE_MODE_FROM_REST = 2,
    // 3 is removed
    // 4 is removed
    APP_STATE_ENTER_CLEANUP_MODE = 5,
    APP_STATE_ENTER_SCALE_CALIBRATION = 6,
    APP_STATE_ENTER_EEPROM_SAVE = 7,
    APP_STATE_ENTER_EEPROM_ERASE = 8,
    APP_STATE_ENTER_REBOOT = 9,
    APP_STATE_ENTER_WIFI_INFO = 10,
    APP_STATE_ENTER_PID_AUTOTUNE_FROM_REST = 11,
    APP_STATE_ENTER_PID_AUTOTUNE = 12,          // started from the encoder menu
    APP_STATE_ENTER_AI_TUNING = 13,             // AI characterization from the encoder menu
    APP_STATE_ENTER_AI_MACHINE_CAL = 14,        // AI machine calibration from the encoder menu
} AppState_t;


typedef struct {
#ifdef _MSC_VER
    // MSVC does not allow a zero-member struct in C. This placeholder only
    // exists for the PC simulator build; the firmware is built with GCC,
    // where this struct stays zero-sized, so the on-device EEPROM layout is
    // completely unaffected by this.
    char _msvc_placeholder;
#endif
} app_persistent_config_t;


typedef struct {
    app_persistent_config_t persistent_config;
} app_config_t;


#ifdef __cplusplus
extern "C" {
#endif

bool app_init();
bool http_app_config();

#ifdef __cplusplus
}
#endif

#endif  // APP_H_

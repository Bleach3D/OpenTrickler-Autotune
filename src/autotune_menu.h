#ifndef AUTOTUNE_MENU_H_
#define AUTOTUNE_MENU_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Encoder-menu entry points; both return the MUI form id to go back to.
uint8_t autotune_pid_menu(void);
uint8_t autotune_ai_menu(bool machine_calibration);

#ifdef __cplusplus
}
#endif

#endif  // AUTOTUNE_MENU_H_

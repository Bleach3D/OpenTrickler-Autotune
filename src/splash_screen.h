#ifndef SPLASH_SCREEN_H_
#define SPLASH_SCREEN_H_

#include "u8g2.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Total length of the animated boot splash.
#define SPLASH_SCREEN_DURATION_MS   10000

// Draws one frame of the animated "OpenTrickler / AUTOTUNE" splash at time
// t_ms (0 .. SPLASH_SCREEN_DURATION_MS) into the u8g2 buffer. Pure function
// of time. Caller clears the buffer before and sends it afterwards.
void splash_screen_draw_frame(u8g2_t *u8g2, uint32_t t_ms);

// Draws the final (static) frame.
void splash_screen_draw(u8g2_t *u8g2);

#ifdef __cplusplus
}
#endif

#endif  // SPLASH_SCREEN_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "u8g2.h"
#include "splash_screen.h"

// Animated boot splash for the 128x64 mini 12864 display:
// "OpenTrickler / AUTOTUNE".
//
// Left:  hopper + trickler tube dropping kernels into a pan on a scale whose
//        readout counts up to the target (coarse, then trickle, then settled).
// Right: name (typed in), an AUTOTUNE badge, and a PID step response that is
//        traced in sync with the charge.
//
// splash_screen_draw_frame() is a pure function of time, so it renders the
// same on the device and in the PC preview. All integer maths, no libm.

// Timeline (ms)
#define T_ICON_IN_END       600     // icon slides in
#define T_TYPE_START        300     // name is typed
#define T_TYPE_CHAR_MS      90
#define T_BADGE_START       1500    // badge wipes in
#define T_BADGE_END         1900
#define T_CHARGE_START      2000    // kernels start falling, readout counts
#define T_COARSE_MS         3000    //   coarse phase length
#define T_FINE_MS           2500    //   trickle phase length
#define T_SETTLED           (T_CHARGE_START + T_COARSE_MS + T_FINE_MS)  // 7500
#define T_FALL_MS           350     // kernel fall time tube -> pan

#define TARGET_CENTI        2430    // readout target: 24.30

#define ICON_X              2

// Icon geometry (relative to the icon origin)
#define HOPPER_CX           18
#define HOPPER_RX           8
#define HOPPER_RY           2
#define HOPPER_BOT          12      // centre of the bottom ellipse
#define HOPPER_LEVEL0       1       // powder surface at the start
#define HOPPER_LEVEL_DROP   2       // ...and how far it drops
#define TUBE_TOP            14
#define TUBE_H              7
#define CUP_TOP             30      // shot glass rim
#define CUP_BOT             43      // shot glass base (bottom row)
#define CUP_RIM_HW          7
#define CUP_BASE_HW         5
#define CUP_FILL_MAX        6       // powder height when full
#define PLATTER_Y           44
#define ICON_Y              2
#define TEXT_X              44


// Charge progress 0..1000 at time t (coarse ease-out to 900, then a cubic
// ease-out "trickle" to exactly 1000).
static int32_t charge_progress(uint32_t t) {
    if (t <= T_CHARGE_START) {
        return 0;
    }
    uint32_t tc = t - T_CHARGE_START;
    if (tc < T_COARSE_MS) {
        int32_t u = (int32_t) (tc * 1000 / T_COARSE_MS);            // 0..1000
        return 900 * u * (2000 - u) / 1000000;                       // ease-out
    }
    tc -= T_COARSE_MS;
    if (tc >= T_FINE_MS) {
        return 1000;
    }
    int32_t r = 1000 - (int32_t) (tc * 1000 / T_FINE_MS);           // 1000..0
    int32_t cube = r * r / 1000 * r / 1000;                          // (1-u)^3
    return 900 + 100 * (1000 - cube) / 1000;
}


// Shot-glass (charge cup) wall x at row y (relative): tapers from the rim
// (CUP_TOP, half width CUP_RIM_HW) to the base (CUP_BOT, CUP_BASE_HW).
static int16_t cup_half_width(int16_t y_rel) {
    int16_t span = CUP_BOT - CUP_TOP;
    return CUP_RIM_HW - (int16_t) ((CUP_RIM_HW - CUP_BASE_HW) * (y_rel - CUP_TOP) / span);
}


static void draw_icon_static(u8g2_t *u8g2, int16_t ox, int16_t oy) {
    // Hopper: upright cylinder running off the top of the screen, with a
    // rounded bottom feeding the trickler tube.
    u8g2_DrawVLine(u8g2, ox + HOPPER_CX - HOPPER_RX, 0, (u8g2_uint_t) (oy + HOPPER_BOT + 1));
    u8g2_DrawVLine(u8g2, ox + HOPPER_CX + HOPPER_RX, 0, (u8g2_uint_t) (oy + HOPPER_BOT + 1));
    u8g2_DrawEllipse(u8g2, ox + HOPPER_CX, oy + HOPPER_BOT, HOPPER_RX, HOPPER_RY,
                     U8G2_DRAW_LOWER_LEFT | U8G2_DRAW_LOWER_RIGHT);

    // Trickler tube (neck below the hopper)
    u8g2_DrawFrame(u8g2, ox + 14, oy + TUBE_TOP, 9, TUBE_H);

    // Shot glass: tapered walls, thick base
    u8g2_DrawLine(u8g2, ox + 18 - CUP_RIM_HW, oy + CUP_TOP, ox + 18 - CUP_BASE_HW, oy + CUP_BOT);
    u8g2_DrawLine(u8g2, ox + 18 + CUP_RIM_HW, oy + CUP_TOP, ox + 18 + CUP_BASE_HW, oy + CUP_BOT);
    u8g2_DrawBox(u8g2, ox + 18 - CUP_BASE_HW, oy + CUP_BOT - 1, CUP_BASE_HW * 2 + 1, 2);

    // Scale: weighing platter + body
    u8g2_DrawBox(u8g2, ox + 7, oy + PLATTER_Y, 23, 2);
    u8g2_DrawRFrame(u8g2, ox + 1, oy + 46, 35, 14, 2);
}


static void draw_icon_dynamic(u8g2_t *u8g2, int16_t ox, int16_t oy, uint32_t t) {
    int32_t p = charge_progress(t);

    // Powder in the hopper (dithered fill); the surface drops as the charge
    // is thrown. The cylinder runs off screen, so the fill starts at the top.
    int16_t level_y = oy + HOPPER_LEVEL0 + (int16_t) (p * HOPPER_LEVEL_DROP / 1000);
    {
        const int16_t xl = ox + HOPPER_CX - HOPPER_RX + 1;
        const int16_t xr = ox + HOPPER_CX + HOPPER_RX - 1;
        for (int16_t y = level_y; y <= oy + HOPPER_BOT + 1; y++) {
            int16_t inset = (y > oy + HOPPER_BOT) ? 2 : 0;   // rounded floor
            if (y == level_y) {
                u8g2_DrawHLine(u8g2, xl, y, (u8g2_uint_t) (xr - xl + 1));
                continue;
            }
            for (int16_t x = xl + inset + ((y + xl + inset) & 1); x <= xr - inset; x += 2) {
                u8g2_DrawPixel(u8g2, x, y);
            }
        }
    }

    // Rotating marker in the tube while it is running
    bool running = (t > T_CHARGE_START && t < T_SETTLED);
    if (running) {
        uint8_t step = (uint8_t) ((t / (t < T_CHARGE_START + T_COARSE_MS ? 80 : 200)) % 4);
        static const int8_t mx[4] = {16, 18, 18, 16};
        static const int8_t my[4] = {0, 0, 2, 2};
        u8g2_DrawBox(u8g2, ox + mx[step], oy + TUBE_TOP + 2 + my[step], 3, 2);
    }
    else {
        u8g2_DrawBox(u8g2, ox + 16, oy + TUBE_TOP + 3, 5, 2);
    }

    // Powder in the shot glass rises with the charge (dithered, solid surface)
    int16_t fill_h = (int16_t) (p * CUP_FILL_MAX / 1000);
    int16_t surface_y = oy + CUP_BOT - 2 - fill_h;          // top of the powder
    for (int16_t y = surface_y + 1; y <= oy + CUP_BOT - 2; y++) {
        int16_t hw = cup_half_width(y - oy) - 1;
        if (y == surface_y + 1) {
            u8g2_DrawHLine(u8g2, ox + 18 - hw, y, (u8g2_uint_t) (2 * hw + 1));
            continue;
        }
        for (int16_t x = ox + 18 - hw + (y & 1); x <= ox + 18 + hw; x += 2) {
            u8g2_DrawPixel(u8g2, x, y);
        }
    }

    // Falling kernels: from the tube mouth down into the glass, landing on
    // the current powder surface. Dense when coarse, sparse when trickling.
    if (t > T_CHARGE_START) {
        int32_t y0 = oy + TUBE_TOP + TUBE_H + 1;
        int32_t d = (surface_y - 1) - y0;
        if (d < 4) {
            d = 4;
        }
        uint32_t coarse_end = T_CHARGE_START + T_COARSE_MS;
        uint32_t window_start = (t > T_FALL_MS) ? t - T_FALL_MS : 0;
        for (int phase = 0; phase < 2; phase++) {
            uint32_t from = phase == 0 ? T_CHARGE_START : coarse_end;
            uint32_t to   = phase == 0 ? coarse_end : T_SETTLED;
            uint32_t period = phase == 0 ? 110 : 420;
            uint32_t first = from;
            if (window_start > from) {
                first = from + ((window_start - from + period - 1) / period) * period;
            }
            for (uint32_t te = first; te < to && te <= t; te += period) {
                uint32_t dt = t - te;                       // 0..T_FALL_MS
                int32_t y = y0 + d * (int32_t) (dt * dt) / (T_FALL_MS * T_FALL_MS);
                int16_t x = ox + 17 + (int16_t) ((te / period) % 3);
                u8g2_DrawBox(u8g2, x, (int16_t) y, 2, 2);
            }
        }
    }

    // Scale readout (inverted window) counting up to the target
    int32_t centi = TARGET_CENTI * p / 1000;
    bool settled = t >= T_SETTLED;
    bool blink_off = settled && t < T_SETTLED + 900 && ((t - T_SETTLED) / 150) % 2 == 1;

    u8g2_DrawBox(u8g2, ox + 4, oy + 49, 24, 8);
    if (!blink_off) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%02ld.%02ld", (long) (centi / 100), (long) (centi % 100));
        u8g2_SetDrawColor(u8g2, 0);
        u8g2_SetFont(u8g2, u8g2_font_4x6_tn);
        u8g2_DrawStr(u8g2, ox + 6, oy + 56, buf);
        u8g2_SetDrawColor(u8g2, 1);
    }

    // Status lamp: hollow while charging, filled (with a tick) when settled
    if (settled) {
        u8g2_DrawDisc(u8g2, ox + 31, oy + 53, 3, U8G2_DRAW_ALL);
        u8g2_SetDrawColor(u8g2, 0);
        u8g2_DrawLine(u8g2, ox + 29, oy + 53, ox + 30, oy + 54);
        u8g2_DrawLine(u8g2, ox + 30, oy + 54, ox + 33, oy + 51);
        u8g2_SetDrawColor(u8g2, 1);
    }
    else {
        u8g2_DrawCircle(u8g2, ox + 31, oy + 53, 3, U8G2_DRAW_ALL);
    }
}


static void draw_step_response(u8g2_t *u8g2, int16_t x0, int16_t y_base, int16_t w, int16_t h,
                               int16_t visible_w) {
    // Setpoint (dotted)
    const int16_t y_sp = y_base - h;
    for (int16_t x = x0; x < x0 + w; x += 3) {
        u8g2_DrawPixel(u8g2, x, y_sp);
    }

    // Underdamped 2nd-order step response (zeta = 0.35), precomputed.
    // Units: 64 = setpoint.
    static const uint8_t resp[] = {
         0,  3, 12, 25, 38, 52, 63, 73, 79, 83, 84, 82, 79, 76, 71, 67,
        64, 61, 59, 58, 58, 58, 59, 61, 62, 63, 64, 65, 66, 66, 66, 66,
        65, 65, 65, 64, 64, 64, 63, 63, 63, 63, 64, 64, 64, 64, 64, 64,
    };
    const int32_t n = (int32_t) (sizeof(resp) / sizeof(resp[0]));
    int16_t prev_y = y_base;
    int16_t head_y = y_base;
    if (visible_w > w) {
        visible_w = w;
    }
    for (int16_t i = 0; i < visible_w; i++) {
        int32_t pos = (int32_t) i * (n - 1) * 256 / (w - 1);
        int32_t k = pos >> 8;
        int32_t f = pos & 0xFF;
        int32_t v = (k + 1 < n) ? (resp[k] * (256 - f) + resp[k + 1] * f) >> 8 : resp[n - 1];
        int16_t y = (int16_t) (y_base - (v * h + 32) / 64);
        if (i == 0) {
            prev_y = y;
        }
        u8g2_DrawLine(u8g2, x0 + i - (i ? 1 : 0), prev_y, x0 + i, y);
        prev_y = y;
        head_y = y;
    }

    // Tracing dot at the head while it is still being drawn
    if (visible_w > 0 && visible_w < w) {
        u8g2_DrawDisc(u8g2, x0 + visible_w - 1, head_y, 1, U8G2_DRAW_ALL);
    }
}


void splash_screen_draw_frame(u8g2_t *u8g2, uint32_t t) {
    const int16_t W = (int16_t) u8g2_GetDisplayWidth(u8g2);
    const int16_t tw = W - TEXT_X - 2;

    u8g2_SetDrawColor(u8g2, 1);
    u8g2_SetFontMode(u8g2, 1);

    // Icon is revealed left to right (ease-out wipe). A wipe rather than a
    // slide: u8g2 coordinates are unsigned, so off-screen x would wrap.
    const int16_t ox = ICON_X;
    if (t < T_ICON_IN_END) {
        int32_t r = 1000 - (int32_t) (t * 1000 / T_ICON_IN_END);    // 1000..0
        int16_t wipe = (int16_t) (42 - 42 * r / 1000 * r / 1000);
        if (wipe <= 0) {
            wipe = 1;
        }
        u8g2_SetClipWindow(u8g2, 0, 0, (u8g2_uint_t) wipe, 64);
    }
    draw_icon_static(u8g2, ox, ICON_Y);
    draw_icon_dynamic(u8g2, ox, ICON_Y, t);
    u8g2_SetMaxClipWindow(u8g2);

    // Name typed in, one character at a time, with a cursor
    static const char name1[] = "Open";
    static const char name2[] = "Trickler";
    int32_t chars = 0;
    if (t > T_TYPE_START) {
        chars = (int32_t) ((t - T_TYPE_START) / T_TYPE_CHAR_MS);
    }
    u8g2_SetFont(u8g2, u8g2_font_helvB10_tr);
    char buf[12];
    int32_t n1 = chars < 4 ? chars : 4;
    int32_t n2 = chars - 4 < 0 ? 0 : (chars - 4 > 8 ? 8 : chars - 4);
    snprintf(buf, sizeof(buf), "%.*s", (int) n1, name1);
    u8g2_DrawStr(u8g2, TEXT_X, 13, buf);
    int16_t cursor_x = TEXT_X + (int16_t) u8g2_GetStrWidth(u8g2, buf);
    int16_t cursor_y = 13;
    if (n2 > 0 || chars >= 4) {
        snprintf(buf, sizeof(buf), "%.*s", (int) n2, name2);
        u8g2_DrawStr(u8g2, TEXT_X, 26, buf);
        cursor_x = TEXT_X + (int16_t) u8g2_GetStrWidth(u8g2, buf);
        cursor_y = 26;
    }
    bool typing = chars < 12;
    if (typing && t > T_TYPE_START && ((t / 120) % 2 == 0)) {
        u8g2_DrawBox(u8g2, cursor_x + 1, cursor_y - 10, 2, 11);
    }

    // AUTOTUNE badge wipes in left to right; flashes once when settled
    if (t >= T_BADGE_START) {
        u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
        const char *badge = "AUTOTUNE";
        int16_t bw = (int16_t) u8g2_GetStrWidth(u8g2, badge) + 6;
        int16_t vis = bw;
        if (t < T_BADGE_END) {
            vis = (int16_t) (bw * (int32_t) (t - T_BADGE_START) / (T_BADGE_END - T_BADGE_START));
        }
        bool flash = t >= T_SETTLED && t < T_SETTLED + 600 && ((t - T_SETTLED) / 150) % 2 == 0;

        u8g2_SetClipWindow(u8g2, TEXT_X, 28, TEXT_X + vis, 44);
        if (flash) {
            u8g2_DrawRFrame(u8g2, TEXT_X, 30, bw, 12, 2);
            u8g2_DrawStr(u8g2, TEXT_X + 3, 39, badge);
        }
        else {
            u8g2_DrawRBox(u8g2, TEXT_X, 30, bw, 12, 2);
            u8g2_SetDrawColor(u8g2, 0);
            u8g2_DrawStr(u8g2, TEXT_X + 3, 39, badge);
            u8g2_SetDrawColor(u8g2, 1);
        }
        u8g2_SetMaxClipWindow(u8g2);
    }

    // Step response traced in sync with the charge
    if (t > T_CHARGE_START) {
        uint32_t span = T_SETTLED - T_CHARGE_START;
        uint32_t tc = t - T_CHARGE_START;
        int16_t visible = tc >= span ? tw : (int16_t) ((int32_t) tw * (int32_t) tc / (int32_t) span);
        draw_step_response(u8g2, TEXT_X, 61, tw, 12, visible);
    }
}


void splash_screen_draw(u8g2_t *u8g2) {
    splash_screen_draw_frame(u8g2, SPLASH_SCREEN_DURATION_MS);
}

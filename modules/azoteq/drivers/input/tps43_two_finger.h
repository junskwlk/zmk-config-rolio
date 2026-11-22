/* SPDX-License-Identifier: MIT */
#ifndef TPS43_TWO_FINGER_H
#define TPS43_TWO_FINGER_H
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>

enum tps43_two_phase {
    TPS43_TWO_IDLE, TPS43_TWO_PENDING, TPS43_TWO_SCROLL,
    TPS43_TWO_NAV_DONE, TPS43_TWO_ENDING, TPS43_TWO_CANCELLED
};
struct tps43_two_finger {
    enum tps43_two_phase phase;
    int64_t started_ms, dx, dy, travel;
    bool tap_possible;
};
struct tps43_two_result {
    bool consume, back, forward;
    int32_t scroll_x, scroll_y;
};
static inline int64_t tps43_two_abs(int64_t value) { return value < 0 ? -value : value; }
static inline int32_t tps43_two_clamp(int64_t value) {
    return value > INT32_MAX ? INT32_MAX : value < INT32_MIN ? INT32_MIN : (int32_t)value;
}
static inline void tps43_two_cancel(struct tps43_two_finger *s) {
    *s = (struct tps43_two_finger){.phase = TPS43_TWO_CANCELLED};
}

/* A quick, horizontal stroke navigates once. Waiting locks scrolling until lift.
 * Vertical intent locks scrolling immediately. Coordinates returned for scrolling
 * stay in sensor space; only navigation classification uses the board transform.
 */
static inline struct tps43_two_result tps43_two_update(
    struct tps43_two_finger *s, uint8_t fingers, int32_t dx, int32_t dy,
    bool scrolling, bool zooming, int64_t now_ms, uint32_t delay_ms,
    uint32_t nav_distance, bool swap_xy, bool invert_horizontal) {
    struct tps43_two_result out = {0};
    if (fingers == 0) {
        out.consume = s->phase != TPS43_TWO_IDLE && !s->tap_possible;
        *s = (struct tps43_two_finger){0};
        return out;
    }
    if (fingers > 2) {
        tps43_two_cancel(s);
        out.consume = true;
        return out;
    }
    if (s->phase == TPS43_TWO_IDLE) {
        if (fingers != 2) { return out; }
        *s = (struct tps43_two_finger){.phase = TPS43_TWO_PENDING,
                                     .started_ms = now_ms, .tap_possible = true};
    }
    out.consume = true;
    if (s->phase == TPS43_TWO_CANCELLED || s->phase == TPS43_TWO_NAV_DONE ||
        s->phase == TPS43_TWO_ENDING) { return out; }
    if (fingers != 2) {
        s->phase = TPS43_TWO_ENDING;
        return out;
    }
    /* Pinches must never become accidental navigation or ordinary scrolling. */
    if (zooming) { tps43_two_cancel(s); return out; }
    if (s->phase == TPS43_TWO_SCROLL) {
        if (scrolling) { out.scroll_x = dx; out.scroll_y = dy; }
        return out;
    }
    if (scrolling) {
        s->dx += dx; s->dy += dy;
        s->travel += tps43_two_abs(dx) + tps43_two_abs(dy);
        if (s->travel >= 12) { s->tap_possible = false; }
    }
    int64_t x = swap_xy ? s->dy : s->dx;
    int64_t y = swap_xy ? s->dx : s->dy;
    if (invert_horizontal) { x = -x; }
    int64_t ax = tps43_two_abs(x), ay = tps43_two_abs(y);
    /* Waiting wins at the exact boundary, even for a large first movement. */
    if (now_ms - s->started_ms >= delay_ms || (ay >= 12 && ay >= 2 * ax)) {
        s->phase = TPS43_TWO_SCROLL;
        s->tap_possible = false;
        out.scroll_x = tps43_two_clamp(s->dx);
        out.scroll_y = tps43_two_clamp(s->dy);
    } else if (ax >= nav_distance && ax >= 2 * ay) {
        s->phase = TPS43_TWO_NAV_DONE;
        s->tap_possible = false;
        out.back = x > 0;
        out.forward = x < 0;
    }
    return out;
}
#endif

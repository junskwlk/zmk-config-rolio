/* SPDX-License-Identifier: MIT */
#ifndef TPS43_GESTURES_H
#define TPS43_GESTURES_H
#include <stdbool.h>
#include <stdint.h>

/* Three-finger motion is always drag; only two short, stationary taps invoke MC.
 * ENDING tolerates fingers lifting at slightly different times without leaking
 * hardware single/two-finger taps during the release of a three-finger chord.
 */
enum tps43_gesture_phase { TPS43_G_IDLE, TPS43_G_PENDING, TPS43_G_DRAG,
                         TPS43_G_ENDING, TPS43_G_CANCELLED };
struct tps43_gesture {
    enum tps43_gesture_phase phase;
    int64_t started_ms, last_tap_ms;
    int64_t dx, dy; /* Total travel, including reversals and staggered finger lift. */
    bool tap_armed;
};
struct tps43_gesture_result {
    bool consume, press, release, move, double_tap;
};
#define TPS43_THREE_FINGER_TAP_MS 220
#define TPS43_THREE_FINGER_DOUBLE_TAP_MS 350

static inline bool tps43_gesture_cancel(struct tps43_gesture *s) {
    bool release = s->phase == TPS43_G_DRAG;
    if (s->phase == TPS43_G_IDLE) {
        s->tap_armed = false;
        return false;
    }
    *s = (struct tps43_gesture){.phase = TPS43_G_CANCELLED};
    return release;
}

static inline struct tps43_gesture_result
tps43_gesture_update(struct tps43_gesture *s, uint8_t fingers, int32_t dx, int32_t dy,
                     int64_t now_ms, uint32_t hold_ms, uint32_t drag_distance) {
    struct tps43_gesture_result out = {0};
    if (s->phase == TPS43_G_PENDING || s->phase == TPS43_G_ENDING) {
        s->dx += dx < 0 ? -(int64_t)dx : dx;
        s->dy += dy < 0 ? -(int64_t)dy : dy;
    }
    if (s->tap_armed && now_ms - s->last_tap_ms > TPS43_THREE_FINGER_DOUBLE_TAP_MS) {
        s->tap_armed = false;
    }
    if (fingers == 0) {
        out.consume = s->phase != TPS43_G_IDLE;
        out.release = s->phase == TPS43_G_DRAG;
        bool tap = (s->phase == TPS43_G_PENDING || s->phase == TPS43_G_ENDING) &&
                   now_ms - s->started_ms <= TPS43_THREE_FINGER_TAP_MS &&
                   s->dx < drag_distance && s->dy < drag_distance;
        if (tap) {
            if (s->tap_armed) {
                out.double_tap = true;
                s->tap_armed = false;
            } else {
                s->tap_armed = true;
                s->last_tap_ms = now_ms;
            }
        } else if (out.consume) {
            s->tap_armed = false;
        }
        s->phase = TPS43_G_IDLE;
        s->dx = s->dy = 0;
        return out;
    }
    if (s->phase == TPS43_G_IDLE) {
        if (fingers < 3) { return out; }
        s->phase = fingers == 3 ? TPS43_G_PENDING : TPS43_G_CANCELLED;
        s->started_ms = now_ms;
        s->dx = dx < 0 ? -(int64_t)dx : dx;
        s->dy = dy < 0 ? -(int64_t)dy : dy;
    }
    out.consume = true;
    if (fingers != 3) {
        out.release = s->phase == TPS43_G_DRAG;
        if (fingers < 3 && (s->phase == TPS43_G_PENDING || s->phase == TPS43_G_ENDING) &&
            s->dx < drag_distance && s->dy < drag_distance) {
            s->phase = TPS43_G_ENDING;
        } else {
            s->phase = TPS43_G_CANCELLED;
            s->tap_armed = false;
        }
        return out;
    }
    if (s->phase == TPS43_G_DRAG) { out.move = true; return out; }
    if (s->phase != TPS43_G_PENDING) { return out; }
    if (s->dx >= drag_distance || s->dy >= drag_distance ||
        now_ms - s->started_ms >= hold_ms) {
        s->phase = TPS43_G_DRAG;
        s->tap_armed = false;
        out.press = out.move = true;
    }
    return out;
}
#endif

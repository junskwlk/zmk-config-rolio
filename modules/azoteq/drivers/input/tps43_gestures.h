/* SPDX-License-Identifier: MIT */
#ifndef TPS43_GESTURES_H
#define TPS43_GESTURES_H

#include <stdbool.h>
#include <stdint.h>

/* Kept independent of Zephyr so recorded input sequences can be tested on a host. */
enum tps43_gesture_phase { TPS43_G_IDLE, TPS43_G_PENDING, TPS43_G_DRAG,
                         TPS43_G_SWIPE, TPS43_G_CANCELLED };
enum tps43_swipe { TPS43_S_NONE, TPS43_S_UP };

struct tps43_gesture {
    enum tps43_gesture_phase phase;
    int64_t started_ms;
    int32_t dx, dy;
};

struct tps43_gesture_result {
    bool consume, press, release, move;
    enum tps43_swipe swipe;
};

static inline bool tps43_gesture_cancel(struct tps43_gesture *s) {
    bool release = s->phase == TPS43_G_DRAG;
    *s = (struct tps43_gesture){.phase = TPS43_G_CANCELLED};
    return release;
}

/* Same bit layout as ZMK INPUT_TRANSFORM_*; affects swipe direction only. */
static inline void tps43_gesture_transform(uint32_t transform, int32_t *x, int32_t *y) {
    if (transform & 1u) {
        int32_t tmp = *x;
        *x = *y;
        *y = tmp;
    }
    if (transform & 2u) { *x = -*x; }
    if (transform & 4u) { *y = -*y; }
}

static inline struct tps43_gesture_result
tps43_gesture_update(struct tps43_gesture *s, uint8_t fingers, int32_t dx, int32_t dy,
                     int64_t now_ms, uint32_t hold_ms, uint32_t distance,
                     uint32_t drag_distance) {
    struct tps43_gesture_result out = {0};
    if (fingers == 0) {
        out.consume = s->phase != TPS43_G_IDLE;
        out.release = s->phase == TPS43_G_DRAG;
        *s = (struct tps43_gesture){0};
        return out;
    }
    if (s->phase == TPS43_G_IDLE) {
        if (fingers < 3) {
            return out;
        }
        s->phase = fingers == 3 ? TPS43_G_PENDING : TPS43_G_CANCELLED;
        s->started_ms = now_ms;
        s->dx = s->dy = 0;
    }
    out.consume = true;
    /* After a partial lift, wait for every finger to lift before accepting input. */
    if (fingers != 3) {
        out.release = s->phase == TPS43_G_DRAG;
        s->phase = TPS43_G_CANCELLED;
        return out;
    }
    if (s->phase == TPS43_G_DRAG) {
        out.move = true;
        return out;
    }
    if (s->phase != TPS43_G_PENDING) {
        return out;
    }

    s->dx += dx;
    s->dy += dy;
    int32_t ax = s->dx < 0 ? -s->dx : s->dx;
    int32_t ay = s->dy < 0 ? -s->dy : s->dy;
    bool upward = s->dy < 0 && ay > ax;
    if (now_ms - s->started_ms < hold_ms && upward && ay >= (int32_t)distance) {
        /* A fast upward stroke is the only reserved swipe. */
        out.swipe = TPS43_S_UP;
        s->phase = TPS43_G_SWIPE;
    } else if (now_ms - s->started_ms >= hold_ms ||
               (!upward && (ax >= (int32_t)drag_distance || ay >= (int32_t)drag_distance))) {
        /* Do not replay movement accumulated before the hold completed. */
        s->phase = TPS43_G_DRAG;
        out.press = true;
        out.move = true;
    }
    return out;
}

#endif

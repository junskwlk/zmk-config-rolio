/* SPDX-License-Identifier: MIT */
#ifndef TPS43_ONE_FINGER_H
#define TPS43_ONE_FINGER_H
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#define TPS43_ONE_TAP_MS 250
#define TPS43_ONE_TAP_DISTANCE 24
#define TPS43_ONE_DOUBLE_TAP_MS 350
#define TPS43_ONE_CHORD_MS 30
#define TPS43_ONE_DRAG_DISTANCE 12

enum tps43_one_phase { TPS43_O_IDLE, TPS43_O_TRACK, TPS43_O_SECOND,
                       TPS43_O_DRAG, TPS43_O_CANCELLED };
struct tps43_one_finger {
    enum tps43_one_phase phase;
    int64_t started_ms, last_tap_ms, dx, dy, excursion;
    bool armed;
};
struct tps43_one_result {
    bool consume, click, press, release, tap_end;
    int32_t move_x, move_y;
    int64_t tap_ms, excursion;
};
static inline int64_t tps43_one_abs(int64_t v) { return v<0?-v:v; }
static inline int32_t tps43_one_clamp(int64_t v) {
    return v>INT32_MAX?INT32_MAX:v<INT32_MIN?INT32_MIN:(int32_t)v;
}
static inline bool tps43_one_cancel(struct tps43_one_finger *s) {
    bool release=s->phase==TPS43_O_DRAG;
    *s=(struct tps43_one_finger){.phase=TPS43_O_CANCELLED};
    return release;
}
/* First tap clicks immediately. A second contact must move far enough before
 * pressing left; elapsed time alone never starts a drag. Keep the chord guard
 * so adding another finger can cancel before a one-finger button press.
 */
static inline struct tps43_one_result tps43_one_update(
    struct tps43_one_finger *s,uint8_t fingers,int32_t dx,int32_t dy,int64_t now) {
    struct tps43_one_result out={.consume=fingers<=1};
    if (fingers>1) {
        out.release=tps43_one_cancel(s);
        return out;
    }
    if (fingers==0) {
        if (s->phase==TPS43_O_TRACK || s->phase==TPS43_O_SECOND) {
            out.tap_end=true;
            out.tap_ms=now-s->started_ms;
            out.excursion=s->excursion;
            bool tap=out.tap_ms<=TPS43_ONE_TAP_MS && s->excursion<=TPS43_ONE_TAP_DISTANCE;
            if (s->phase==TPS43_O_TRACK) {
                out.click=tap;
                s->armed=tap;
                s->last_tap_ms=now;
            } else {
                if (s->excursion>=TPS43_ONE_DRAG_DISTANCE) {
                    out.press=out.release=true;
                    out.move_x=tps43_one_clamp(s->dx);
                    out.move_y=tps43_one_clamp(s->dy);
                } else if (tap) { out.click=true; }
                s->armed=false;
            }
        } else {
            out.release=s->phase==TPS43_O_DRAG;
            if (s->phase!=TPS43_O_IDLE) { s->armed=false; }
        }
        s->phase=TPS43_O_IDLE;
        s->dx=s->dy=s->excursion=0;
        return out;
    }
    if (s->phase==TPS43_O_CANCELLED) { return out; }
    if (s->phase==TPS43_O_IDLE) {
        bool second=s->armed && now-s->last_tap_ms<TPS43_ONE_DOUBLE_TAP_MS;
        *s=(struct tps43_one_finger){.phase=second?TPS43_O_SECOND:TPS43_O_TRACK,
                                     .started_ms=now};
    }
    if (s->phase==TPS43_O_DRAG) {
        out.move_x=dx;out.move_y=dy;
        return out;
    }
    s->dx+=dx;s->dy+=dy;
    int64_t x=tps43_one_abs(s->dx),y=tps43_one_abs(s->dy);
    if (x>s->excursion) { s->excursion=x; }
    if (y>s->excursion) { s->excursion=y; }
    if (s->phase==TPS43_O_TRACK) {
        out.move_x=dx;out.move_y=dy;
    } else if (now-s->started_ms>=TPS43_ONE_CHORD_MS &&
               s->excursion>=TPS43_ONE_DRAG_DISTANCE) {
        s->phase=TPS43_O_DRAG;
        out.press=true;
        out.move_x=tps43_one_clamp(s->dx);
        out.move_y=tps43_one_clamp(s->dy);
        s->dx=s->dy=0;
    }
    return out;
}
#endif

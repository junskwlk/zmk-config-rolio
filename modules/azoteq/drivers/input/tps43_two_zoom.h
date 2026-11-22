/* SPDX-License-Identifier: MIT */
#ifndef TPS43_TWO_ZOOM_H
#define TPS43_TWO_ZOOM_H
#include <stdbool.h>
#include <stdint.h>
#define TPS43_TWO_ZOOM_TAP_MS 220
#define TPS43_TWO_ZOOM_GAP_MS 300
#define TPS43_TWO_ZOOM_TAP_TRAVEL 12

enum tps43_two_zoom_phase {
    TPS43_Z_IDLE, TPS43_Z_FIRST, TPS43_Z_ENDING, TPS43_Z_ACTIVE, TPS43_Z_CANCELLED
};
struct tps43_two_zoom {
    enum tps43_two_zoom_phase phase;
    int64_t started_ms, click_due_ms, travel;
    bool pending_click, tap_possible;
};
struct tps43_two_zoom_result { bool consume, click, move; };
static inline void tps43_two_zoom_cancel(struct tps43_two_zoom *s) {
    *s=(struct tps43_two_zoom){.phase=TPS43_Z_CANCELLED};
}
static inline bool tps43_two_zoom_tick(struct tps43_two_zoom *s, int64_t now_ms) {
    if (!s->pending_click || now_ms < s->click_due_ms) { return false; }
    s->pending_click=false;
    return true;
}
/* This classifier owns tap recognition, not the hardware TWO_FINGER_TAP flag.
 * A first short tap defers right click; second two-finger contact enters zoom.
 * Staggered finger lifts cannot restart zoom before a complete release.
 */
static inline struct tps43_two_zoom_result tps43_two_zoom_update(
    struct tps43_two_zoom *s, uint8_t fingers, int32_t dx, int32_t dy, int64_t now_ms) {
    struct tps43_two_zoom_result out={0};
    if (fingers>2) {
        tps43_two_zoom_cancel(s);
        out.consume=true;
        return out;
    }
    out.click=tps43_two_zoom_tick(s,now_ms);
    if (fingers==0) {
        out.consume=s->pending_click;
        if ((s->phase==TPS43_Z_FIRST || s->phase==TPS43_Z_ENDING) &&
            s->tap_possible && now_ms-s->started_ms<=TPS43_TWO_ZOOM_TAP_MS) {
            s->pending_click=true;
            s->click_due_ms=now_ms+TPS43_TWO_ZOOM_GAP_MS;
            out.consume=true;
        } else if (s->phase==TPS43_Z_ACTIVE || s->phase==TPS43_Z_CANCELLED) {
            out.consume=true;
        }
        s->phase=TPS43_Z_IDLE;
        s->tap_possible=false;
        return out;
    }
    if (s->phase==TPS43_Z_CANCELLED) { out.consume=true; return out; }
    if (s->phase==TPS43_Z_ACTIVE) {
        out.consume=true;
        if (fingers==2) { out.move=true; }
        else { s->phase=TPS43_Z_CANCELLED; }
        return out;
    }
    if (s->phase==TPS43_Z_IDLE) {
        if (fingers!=2) { out.consume=s->pending_click; return out; }
        if (s->pending_click) {
            s->pending_click=false;
            s->phase=TPS43_Z_ACTIVE;
            out.consume=true; out.move=true;
            return out;
        }
        s->phase=TPS43_Z_FIRST;
        s->started_ms=now_ms;
        s->travel=0;
        s->tap_possible=true;
    }
    if (s->phase==TPS43_Z_ENDING) { out.consume=s->tap_possible; return out; }
    if (fingers!=2) {
        s->phase=TPS43_Z_ENDING;
        out.consume=s->tap_possible;
        return out;
    }
    s->travel+=(dx<0?-(int64_t)dx:dx)+(dy<0?-(int64_t)dy:dy);
    if (s->travel>=TPS43_TWO_ZOOM_TAP_TRAVEL || now_ms-s->started_ms>TPS43_TWO_ZOOM_TAP_MS) {
        s->tap_possible=false;
    }
    return out;
}
#endif

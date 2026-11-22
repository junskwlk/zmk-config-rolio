/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <stdio.h>
#include "../modules/azoteq/drivers/input/tps43_two_zoom.h"
static struct tps43_two_zoom_result step(struct tps43_two_zoom *s,int f,int x,int y,int t) {
    return tps43_two_zoom_update(s,f,x,y,t);
}
static void tap(struct tps43_two_zoom *s,int t) {
    step(s,2,0,0,t); assert(step(s,0,0,0,t+50).consume);
    assert(s->pending_click);
}
int main(void) {
    struct tps43_two_zoom s={0};struct tps43_two_zoom_result r;
    /* One stationary tap is one right click, only after the full gap. */
    assert(!step(&s,1,10,0,0).consume);
    tap(&s,100);assert(!tps43_two_zoom_tick(&s,449));
    assert(tps43_two_zoom_tick(&s,450));assert(!tps43_two_zoom_tick(&s,451));
    /* A second contact starts zoom, without sending either right click. */
    tap(&s,1000);r=step(&s,2,-20,0,1349);
    assert(r.consume && r.move && !r.click && !s.pending_click);
    r=step(&s,2,20,0,1400);assert(r.move && !r.click);
    assert(!tps43_two_zoom_tick(&s,1500));
    assert(step(&s,1,0,0,1510).consume); /* first lifted finger ends zoom */
    r=step(&s,2,-30,0,1520);assert(r.consume && !r.move);
    step(&s,0,0,0,1530);assert(s.phase==TPS43_Z_IDLE);
    /* At exactly the deadline, first click completes; new tap is separate. */
    tap(&s,2000);r=step(&s,2,0,0,2350);assert(r.click && !r.move);
    step(&s,0,0,0,2400);assert(s.pending_click);
    assert(tps43_two_zoom_tick(&s,2700));
    /* Staggered lifts preserve a short tap, without duplicate clicks. */
    step(&s,2,0,0,3000);assert(step(&s,1,0,0,3020).consume);
    step(&s,0,0,0,3040);assert(s.pending_click);
    r=step(&s,1,0,0,3100);assert(r.consume && !r.move);
    r=step(&s,2,0,0,3110);assert(r.move);
    step(&s,0,0,0,3150);assert(!s.pending_click);
    /* Ordinary scrolling never arms zoom or delays its own output. */
    step(&s,2,0,0,4000);r=step(&s,2,12,0,4010);assert(!r.consume && !r.move);
    step(&s,1,0,0,4020);step(&s,0,0,0,4030);assert(!s.pending_click);
    r=step(&s,2,0,0,4100);assert(!r.move);step(&s,0,0,0,4400);
    assert(!s.pending_click); /* long hold is not a tap */
    /* Any three/four-finger contact cancels the pending right click/zoom. */
    for(int n=3;n<=4;n++) {
        tap(&s,5000);r=step(&s,n,0,0,5100);assert(r.consume && !r.click);
        assert(!s.pending_click);r=step(&s,2,0,0,5200);assert(!r.move);
        step(&s,0,0,0,5300);
    }
    tap(&s,6000);tps43_two_zoom_cancel(&s);assert(!tps43_two_zoom_tick(&s,7000));
    step(&s,0,0,0,7010);
    /* Full int32 deltas are safely counted as movement, never a false tap. */
    step(&s,2,INT32_MIN,INT32_MAX,8000);step(&s,0,0,0,8010);assert(!s.pending_click);
    step(&s,2,0,0,9000);step(&s,0,0,0,9220);assert(s.pending_click);
    assert(step(&s,0,0,0,9221).consume);assert(tps43_two_zoom_tick(&s,9520));
    step(&s,2,0,0,10000);step(&s,0,0,0,10221);assert(!s.pending_click);
    puts("two-finger tap/right-click delay/hold-zoom/isolation tests passed");
}

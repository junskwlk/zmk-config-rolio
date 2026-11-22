/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <stdio.h>
#include "../modules/azoteq/drivers/input/tps43_two_finger.h"
/* screen X=-sensor Y, screen Y=sensor X */
static struct tps43_two_result step(struct tps43_two_finger *s, unsigned fingers,
                                  int x, int y, int ms) {
    return tps43_two_update(s,fingers,y,-x,true,false,ms,250,80,true,true);
}
static void lift(struct tps43_two_finger *s, int ms) { step(s,0,0,0,ms); }
static void silent(struct tps43_two_result r) {
    assert(!r.back && !r.forward && !r.scroll_x && !r.scroll_y);
}
int main(void) {
    struct tps43_two_finger s={0}; struct tps43_two_result r;
    assert(!step(&s,1,50,20,0).consume);
    /* Navigation waits through the guard, then fires exactly once. */
    silent(step(&s,2,40,0,10)); silent(step(&s,2,40,0,30));
    silent(step(&s,2,0,0,89)); r=step(&s,2,0,0,90); assert(r.back);
    for(int i=0;i<10;i++) silent(step(&s,2,i%2?-200:200,0,100+i*100));
    assert(step(&s,1,0,0,1100).consume); lift(&s,1110);
    silent(step(&s,2,-100,0,1200)); r=step(&s,2,0,0,1280); assert(r.forward);
    lift(&s,1300);
    /* A pinch after initially scroll-like motion discards all buffered motion. */
    silent(step(&s,2,0,20,2000)); silent(step(&s,2,0,10,2079));
    r=tps43_two_update(&s,2,5,0,true,true,2080,250,80,true,true); silent(r);
    assert(s.phase==TPS43_TWO_CANCELLED);
    silent(step(&s,2,0,500,2100)); lift(&s,2110);
    /* Guard begins on movement even after a long stationary hold. */
    silent(step(&s,2,0,0,2200)); silent(step(&s,2,0,0,2600));
    assert(s.phase==TPS43_TWO_PENDING);
    silent(step(&s,2,100,0,2700)); silent(step(&s,2,0,0,2779));
    r=step(&s,2,0,0,2780); assert(!r.back && r.scroll_y==-100);
    r=step(&s,2,-200,0,2790); assert(!r.forward && r.scroll_y==200);
    lift(&s,2800);
    /* Exact 250ms boundary still selects scroll over navigation. */
    silent(step(&s,2,0,0,3000)); silent(step(&s,2,100,0,3169));
    r=step(&s,2,0,0,3249); assert(r.back); lift(&s,3250);
    silent(step(&s,2,0,0,3300)); silent(step(&s,2,100,0,3470));
    r=step(&s,2,0,0,3550); assert(!r.back && r.scroll_y==-100);
    silent(step(&s,2,0,0,3560)); lift(&s,3570);
    /* Vertical scroll waits 80ms, flushes once, then has no added delay. */
    silent(step(&s,2,0,12,4000)); silent(step(&s,2,0,0,4079));
    r=step(&s,2,0,0,4080); assert(r.scroll_x==12 && !r.back);
    r=step(&s,2,100,0,4090); assert(r.scroll_y==-100 && !r.back);
    lift(&s,4100);
    silent(step(&s,2,100,80,4200)); silent(step(&s,2,0,0,4280));
    r=step(&s,2,0,0,4450); assert(r.scroll_x==80 && r.scroll_y==-100);
    lift(&s,4460);
    /* Short taps pass; quick strokes flush once on lift, including final pinch veto. */
    step(&s,2,0,0,5000); step(&s,1,0,0,5040);
    assert(!step(&s,0,0,0,5050).consume);
    silent(step(&s,2,0,30,5100)); r=step(&s,0,0,0,5150);
    assert(r.consume && r.scroll_x==30); silent(step(&s,0,0,0,5160));
    silent(step(&s,2,100,0,5170)); r=step(&s,0,0,0,5180); assert(r.back);
    silent(step(&s,2,-100,0,5190)); r=step(&s,0,0,0,5195); assert(r.forward);
    silent(step(&s,2,0,30,5196));
    r=tps43_two_update(&s,0,0,0,false,true,5197,250,80,true,true); silent(r);
    silent(step(&s,2,0,30,5198)); r=step(&s,1,0,0,5199);
    assert(r.scroll_x==30); silent(step(&s,1,0,0,5200)); silent(step(&s,0,0,0,5201));
    /* Finger count changes cannot restart navigation before complete lift. */
    step(&s,2,30,0,5200); step(&s,1,0,0,5210);
    silent(step(&s,2,200,0,5300)); lift(&s,5310);
    for(unsigned fingers=3;fingers<=4;fingers++) {
        step(&s,2,30,0,5400); step(&s,fingers,0,0,5410);
        silent(step(&s,2,200,0,5500)); lift(&s,5510);
    }
    /* Cancellation, non-scroll reports and generic coordinate configuration. */
    step(&s,2,20,0,6000); tps43_two_cancel(&s);
    silent(step(&s,2,200,0,6100)); lift(&s,6110);
    silent(tps43_two_update(&s,2,0,-200,false,false,6200,250,80,true,true));
    assert(!s.motion_started); lift(&s,6210);
    silent(tps43_two_update(&s,2,100,0,true,false,6300,250,80,false,false));
    r=tps43_two_update(&s,2,0,0,true,false,6380,250,80,false,false); assert(r.back);
    lift(&s,6400);
    silent(tps43_two_update(&s,2,INT32_MIN,0,true,false,6500,250,80,false,false));
    r=tps43_two_update(&s,2,0,0,true,false,6580,250,80,false,false); assert(r.forward);
    lift(&s,6600);
    step(&s,2,0,0,6700);
    silent(tps43_two_update(&s,2,INT32_MAX,INT32_MIN,true,false,7000,250,80,true,true));
    r=step(&s,2,0,0,7080); assert(r.scroll_x==INT32_MAX && r.scroll_y==INT32_MIN);
    puts("guarded navigation / delayed scrolling / pinch precedence tests passed");
}

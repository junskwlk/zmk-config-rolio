/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <stdio.h>
#include "../modules/azoteq/drivers/input/tps43_two_finger.h"

/* The Sofle listener uses screen X=-sensor Y and screen Y=sensor X. */
static struct tps43_two_result step(struct tps43_two_finger *s, unsigned fingers,
                                  int x, int y, int ms) {
    return tps43_two_update(s, fingers, y, -x, true, false, ms, 250, 80, true, true);
}
static void lift(struct tps43_two_finger *s, int ms) { step(s, 0, 0, 0, ms); }
int main(void) {
    struct tps43_two_finger s = {0};
    struct tps43_two_result r;
    assert(!step(&s,1,50,20,0).consume);
    r=step(&s,2,40,0,10); assert(r.consume && !r.back && !r.scroll_y);
    r=step(&s,2,40,0,30); assert(r.back && !r.forward && !r.scroll_y);
    /* One action per contact, including direction reversals and later pauses. */
    for (int i=0;i<10;i++) {
        r=step(&s,2,i%2 ? -200:200,0,50+i*100);
        assert(!r.back && !r.forward && !r.scroll_x && !r.scroll_y);
    }
    assert(step(&s,1,0,0,1100).consume);
    assert(step(&s,0,0,0,1110).consume);
    r=step(&s,2,-100,0,1200); assert(r.forward && !r.back); lift(&s,1210);

    /* Waiting, including without intervening sensor frames, selects scrolling. */
    step(&s,2,0,0,2000);
    r=step(&s,2,100,0,2250);
    assert(!r.back && r.scroll_x==0 && r.scroll_y==-100);
    r=step(&s,2,-200,0,2260); assert(!r.forward && r.scroll_y==200);
    lift(&s,2270);
    step(&s,2,0,0,2300);
    r=step(&s,2,100,0,2549); assert(r.back); lift(&s,2560);
    step(&s,2,40,0,2600);
    r=step(&s,2,0,0,2900); assert(!r.back && r.scroll_y==-40);
    r=step(&s,2,0,0,2910); assert(!r.scroll_y); lift(&s,2920);

    /* Vertical scrolling starts immediately and locks out navigation. */
    r=step(&s,2,0,12,3000); assert(r.scroll_x==12 && !r.back);
    r=step(&s,2,100,0,3010); assert(r.scroll_y==-100 && !r.back);
    lift(&s,3020);
    /* Ambiguous diagonals never navigate; buffered motion scrolls after waiting. */
    r=step(&s,2,100,80,3100); assert(!r.back && !r.scroll_x && !r.scroll_y);
    r=step(&s,2,0,0,3350); assert(r.scroll_x==80 && r.scroll_y==-100);
    lift(&s,3360);

    /* An unmoved short tap passes to hardware right-click handling. */
    step(&s,2,0,0,4000); step(&s,1,0,0,4040);
    assert(!step(&s,0,0,0,4050).consume);
    step(&s,2,15,0,4100); assert(step(&s,0,0,0,4150).consume);
    /* A finger-count change cannot restart navigation before complete lift. */
    step(&s,2,30,0,4200); step(&s,1,0,0,4210);
    r=step(&s,2,200,0,4220); assert(!r.back); lift(&s,4230);
    for (unsigned fingers=3;fingers<=4;fingers++) {
        step(&s,2,30,0,4300); step(&s,fingers,0,0,4310);
        r=step(&s,2,200,0,4320); assert(!r.back && !r.scroll_y);
        lift(&s,4330);
    }
    /* Pinch flags and driver cancellation never leak into navigation. */
    tps43_two_update(&s,2,0,-200,true,true,5000,250,80,true,true);
    assert(!step(&s,2,200,0,5010).back); lift(&s,5020);
    step(&s,2,20,0,5100); tps43_two_cancel(&s);
    assert(!step(&s,2,200,0,5110).back); lift(&s,5120);
    /* Non-scroll sensor motion does not become a browser shortcut. */
    r=tps43_two_update(&s,2,0,-200,false,false,5200,250,80,true,true);
    assert(!r.back && !r.scroll_y); lift(&s,5210);
    /* Generic axis configuration and extreme raw values remain well-defined. */
    r=tps43_two_update(&s,2,100,0,true,false,6000,250,80,false,false);
    assert(r.back); lift(&s,6010);
    r=tps43_two_update(&s,2,INT32_MIN,0,true,false,6100,250,80,false,false);
    assert(r.forward); lift(&s,6110);
    step(&s,2,0,0,6200);
    r=tps43_two_update(&s,2,INT32_MAX,INT32_MIN,true,false,6500,250,80,true,true);
    assert(r.scroll_x==INT32_MAX && r.scroll_y==INT32_MIN);
    puts("two-finger quick navigation / delayed scrolling tests passed");
}

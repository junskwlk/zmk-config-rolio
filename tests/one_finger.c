/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <stdio.h>
#include "../modules/azoteq/drivers/input/tps43_one_finger.h"
static struct tps43_one_result step(struct tps43_one_finger *s,int f,int x,int y,int ms) {
    return tps43_one_update(s,f,x,y,ms);
}
int main(void) {
    struct tps43_one_finger s={0};struct tps43_one_result r;
    /* First tap clicks immediately on release; no hardware gesture flag needed. */
    r=step(&s,1,0,0,100);assert(r.consume && !r.click && !r.press);
    r=step(&s,0,0,0,180);assert(r.click && r.tap_end && r.tap_ms==80 && s.armed);
    assert(!step(&s,0,0,0,181).click);assert(s.armed);
    /* Second touch holds left click before movement is emitted, then releases. */
    r=step(&s,1,2,0,300);assert(!r.press && !r.move_x);
    r=step(&s,1,3,4,329);assert(!r.press && !r.move_x);
    r=step(&s,1,5,6,330);assert(!r.press && !r.move_x);
    r=step(&s,1,2,0,331);assert(r.press && r.move_x==12 && r.move_y==10);
    r=step(&s,1,-8,2,350);assert(!r.press && r.move_x==-8 && r.move_y==2);
    r=step(&s,0,0,0,400);assert(r.release && !r.click && !s.armed);
    assert(!step(&s,0,0,0,410).release);
    /* A normal short second tap is a second click, preserving double-click. */
    step(&s,1,0,0,1000);assert(step(&s,0,0,0,1050).click);
    step(&s,1,0,0,1100);r=step(&s,0,0,0,1110);assert(r.click && !r.press && !s.armed);
    /* Replacing a finger on an open menu never presses solely due to time. */
    step(&s,1,0,0,1200);assert(step(&s,0,0,0,1250).click);
    step(&s,1,0,0,1300);
    r=step(&s,1,0,0,1330);assert(!r.press);
    r=step(&s,1,0,0,1400);assert(!r.press);
    r=step(&s,0,0,0,1450);assert(r.click && !r.release);
    /* A stationary long second contact produces no drag or click. */
    step(&s,1,0,0,1500);step(&s,0,0,0,1550);
    step(&s,1,0,0,1600);
    r=step(&s,1,0,0,1900);assert(!r.press);
    r=step(&s,0,0,0,1901);assert(!r.click && !r.press && !r.release);
    /* Subthreshold jitter stays below the drag gate, even beyond the guard. */
    step(&s,1,0,0,1920);step(&s,0,0,0,1930);
    step(&s,1,11,0,1940);r=step(&s,1,-11,0,1970);assert(!r.press);
    r=step(&s,0,0,0,1980);assert(r.click && !r.release);
    /* Small jitter is not summed repeatedly into a false drag rejection. */
    step(&s,1,0,0,2000);
    for(int i=0;i<10;i++) step(&s,1,i%2?-10:10,0,2010+i*10);
    assert(step(&s,0,0,0,2200).click);
    /* Exact duration and distance limits. Ordinary cursor movement stays immediate. */
    tps43_one_cancel(&s);step(&s,0,0,0,2300);
    r=step(&s,1,24,0,2400);assert(r.move_x==24);
    assert(step(&s,0,0,0,2650).click);
    tps43_one_cancel(&s);step(&s,0,0,0,2700);
    step(&s,1,0,0,2800);assert(!step(&s,0,0,0,3051).click);
    r=step(&s,1,25,0,3100);assert(r.move_x==25);
    assert(!step(&s,0,0,0,3150).click);
    /* A single long touch does not begin the old press-and-hold drag. */
    step(&s,1,0,0,4000);r=step(&s,1,10,0,4400);assert(!r.press && r.move_x==10);
    r=step(&s,0,0,0,4500);assert(!r.click && !r.release);
    /* Two/three fingers arriving during chord assembly produce no left click. */
    for(int n=2;n<=4;n++) {
        step(&s,1,0,0,5000);assert(step(&s,0,0,0,5050).click);
        step(&s,1,3,0,5100);r=step(&s,n,0,0,5110);assert(!r.press && !r.release && !r.consume);
        r=step(&s,1,20,0,5150);assert(!r.move_x && !r.click);
        assert(!step(&s,0,0,0,5200).click);
    }
    /* A multi-finger chord or an error cancels an already-held left click once. */
    step(&s,1,0,0,6000);step(&s,0,0,0,6050);
    step(&s,1,0,0,6100);assert(step(&s,1,12,0,6130).press);
    r=step(&s,2,0,0,6200);assert(r.release && !r.consume);step(&s,0,0,0,6250);
    step(&s,1,0,0,6300);step(&s,0,0,0,6350);
    step(&s,1,0,0,6400);step(&s,1,0,12,6430);
    assert(tps43_one_cancel(&s));assert(!tps43_one_cancel(&s));step(&s,0,0,0,6500);
    /* 199ms permits drag; 200ms and later retouches only move the cursor. */
    for (int gap=199;gap<=350;gap++) {
        s=(struct tps43_one_finger){0};
        step(&s,1,0,0,6600);assert(step(&s,0,0,0,6650).click);
        r=step(&s,1,12,0,6650+gap);
        assert(!r.press);
        r=step(&s,1,1,0,6680+gap);
        if (gap==199) { assert(r.press && r.move_x==13); }
        else { assert(!r.press && r.move_x==1 && s.phase==TPS43_O_TRACK); }
        step(&s,0,0,0,6690+gap);
    }
    s=(struct tps43_one_finger){0};
    /* Gap equality is a new first contact, not double tap. */
    step(&s,1,0,0,7000);step(&s,0,0,0,7050);
    r=step(&s,1,0,0,7250);assert(s.phase==TPS43_O_TRACK && !r.press);
    assert(step(&s,0,0,0,7450).click);
    /* A rapid drag that finishes during chord guard preserves movement and release. */
    step(&s,1,50,0,7500);r=step(&s,0,0,0,7510);
    assert(r.press && r.move_x==50 && r.release && !r.click);
    step(&s,1,INT32_MIN,INT32_MAX,8000);assert(!step(&s,0,0,0,8010).click);
    puts("immediate click / movement-gated drag / stationary retouch / chord isolation tests passed");
}

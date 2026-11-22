/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <stdio.h>
#include "../modules/azoteq/drivers/input/tps43_gestures.h"
static struct tps43_gesture_result step(struct tps43_gesture *s, unsigned fingers,
                                       int x, int y, int ms) {
    return tps43_gesture_update(s, fingers, x, y, ms, 250, 12);
}
int main(void) {
    struct tps43_gesture s = {0};
    struct tps43_gesture_result r;
    assert(!step(&s, 1, 100, 100, 0).consume);
    assert(!step(&s, 2, 100, 100, 1).consume);
    assert(!tps43_gesture_cancel(&s));

    /* Every initial direction starts a drag, including fast straight up. */
    int xy[4][2] = {{0,-200}, {0,200}, {-200,0}, {200,0}};
    int t = 100;
    for (int i=0; i<4; i++,t+=1000) {
        r=step(&s,3,xy[i][0],xy[i][1],t);
        assert(r.press && r.move && !r.double_tap);
        for (int j=0; j<4; j++) {
            r=step(&s,3,xy[j][0],xy[j][1],t+10+j);
            assert(r.move && !r.press && !r.release && !r.double_tap);
        }
        assert(step(&s,0,0,0,t+50).release);
        assert(!step(&s,0,0,0,t+60).release);
    }
    /* First tap never clicks/opens MC; second tap opens exactly once on lift. */
    step(&s,3,0,0,5000);
    r=step(&s,2,0,0,5050); assert(r.consume && !r.press && !r.double_tap);
    step(&s,1,0,0,5060);
    r=step(&s,0,0,0,5070); assert(r.consume && !r.double_tap && !r.release);
    assert(!step(&s,0,0,0,5100).double_tap);
    step(&s,1,0,0,5150); step(&s,2,0,0,5160);
    r=step(&s,3,0,0,5170); assert(!r.press && !r.double_tap);
    step(&s,2,0,0,5200);
    r=step(&s,0,0,0,5220); assert(r.double_tap && !r.press && !r.release);
    assert(!step(&s,0,0,0,5230).double_tap);
    step(&s,3,0,0,5300); assert(!step(&s,0,0,0,5350).double_tap);
    /* Timeouts do not convert two separate taps to a double tap. */
    step(&s,3,0,0,6000); assert(!step(&s,0,0,0,6050).double_tap);
    step(&s,3,0,0,6500); assert(!step(&s,0,0,0,6550).double_tap);
    /* A tap followed by movement is drag, not Mission Control. */
    step(&s,3,0,0,6600);
    r=step(&s,3,0,-15,6610); assert(r.press && r.move && !r.double_tap);
    assert(step(&s,2,0,0,6620).release);
    assert(!step(&s,3,0,-200,6630).press);
    assert(!step(&s,0,0,0,6650).double_tap);
    /* Stationary hold starts drag, and is never counted as a short tap. */
    step(&s,3,0,0,7000);
    assert(step(&s,3,0,0,7250).press);
    assert(step(&s,3,0,-50,7260).move);
    assert(tps43_gesture_cancel(&s));
    assert(!step(&s,3,0,-100,7270).move);
    step(&s,0,0,0,7280);
    step(&s,3,0,0,7500); assert(!step(&s,0,0,0,7750).double_tap);
    /* Fourth finger cancels tap and drag; full lift rearms normal pointing. */
    step(&s,3,0,0,8000); step(&s,4,0,0,8010);
    assert(!step(&s,0,0,0,8020).double_tap);
    step(&s,3,30,0,8100); assert(step(&s,4,0,0,8110).release);
    step(&s,0,0,0,8120);
    assert(!step(&s,1,10,10,8130).consume);
    puts("all-direction drag and three-finger double-tap tests passed");
}

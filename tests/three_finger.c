/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <stdio.h>
#include "../modules/azoteq/drivers/input/tps43_gestures.h"

static struct tps43_gesture_result step(struct tps43_gesture *s, unsigned fingers,
                                       int x, int y, int ms) {
    return tps43_gesture_update(s, fingers, x, y, ms, 250, 150, 20);
}

int main(void) {
    struct tps43_gesture s = {0};
    struct tps43_gesture_result r;

    /* Single-finger motion and two-finger scrolling stay with the existing driver. */
    assert(!step(&s, 1, 20, -20, 0).consume);
    assert(!step(&s, 2, 20, -20, 10).consume);
    /* I2C errors with no active gesture must leave normal pointing available. */
    assert(!tps43_gesture_cancel(&s));
    assert(s.phase == TPS43_G_IDLE);
    assert(!step(&s, 1, 20, -20, 15).consume);

    /* Accumulate a quick upward swipe; emit once, with no cursor movement. */
    r = step(&s, 3, 0, -70, 20);
    assert(r.consume && !r.move && !r.swipe && !r.press);
    r = step(&s, 3, 10, -90, 40);
    assert(r.swipe == TPS43_S_UP && !r.move && !r.press);
    for (int t = 50; t < 500; t += 10) {
        r = step(&s, 3, 0, -100, t);
        assert(!r.swipe && !r.press && !r.move);
    }
    /* Partial lifts must not rearm a gesture or leak scroll/click events. */
    assert(step(&s, 2, 20, 20, 510).consume);
    assert(!step(&s, 3, 0, -300, 520).swipe);
    assert(step(&s, 0, 0, 0, 530).consume);
    assert(!step(&s, 1, 10, 0, 540).consume);

    /* Hold, then drag; moving upward during the drag never opens Mission Control. */
    r = step(&s, 3, 0, 0, 600);
    assert(!r.press && !r.move);
    r = step(&s, 3, 1, -1, 849);
    assert(!r.press && !r.move);
    r = step(&s, 3, 0, 0, 850);
    assert(r.press && r.move && !r.swipe);
    r = step(&s, 3, 0, -200, 860);
    assert(r.move && !r.press && !r.swipe);
    r = step(&s, 2, 1, 0, 870);
    assert(r.release && !r.move);
    assert(!step(&s, 1, 100, 100, 880).release);
    assert(!step(&s, 3, 0, 0, 900).press);
    step(&s, 0, 0, 0, 910);

    /* Full lift releases once, even without a final movement sample. */
    step(&s, 3, 0, 0, 1000);
    assert(step(&s, 3, 0, 0, 1250).press);
    assert(step(&s, 0, 0, 0, 1260).release);
    assert(!step(&s, 0, 0, 0, 1270).release);

    /* A cancelled/tiny gesture cannot click or become an accidental drag. */
    step(&s, 3, 0, 0, 1300);
    assert(step(&s, 2, 0, 0, 1310).consume);
    assert(!step(&s, 3, 0, 0, 1700).press);
    assert(!step(&s, 0, 0, 0, 1710).release);

    /* Sideways/downward tracing starts dragging, never emits Mission Control. */
    r = step(&s, 3, 25, 0, 1800);
    assert(r.press && r.move && !r.swipe);
    assert(step(&s, 3, 0, -500, 1810).move);
    assert(step(&s, 0, 0, 0, 1820).release);
    r = step(&s, 3, -25, 0, 1900);
    assert(r.press && r.move && !r.swipe);
    step(&s, 0, 0, 0, 1910);
    r = step(&s, 3, 0, 25, 2000);
    assert(r.press && r.move && !r.swipe);
    step(&s, 0, 0, 0, 2010);

    /* Adding a fourth finger during drag releases; no restart until full lift. */
    step(&s, 3, 0, 0, 2100);
    assert(step(&s, 3, 0, 0, 2350).press);
    assert(step(&s, 4, 0, 0, 2360).release);
    assert(!step(&s, 3, 0, 0, 2700).press);
    step(&s, 0, 0, 0, 2710);
    assert(step(&s, 4, 0, -500, 2800).consume);
    assert(!step(&s, 3, 0, -500, 2810).swipe);
    step(&s, 0, 0, 0, 2820);

    /* Raw direction must match the existing central swap/invert transformation. */
    int32_t x = -160, y = 0;
    tps43_gesture_transform(3, &x, &y);
    assert(x == 0 && y == -160);
    assert(step(&s, 3, x, y, 2900).swipe == TPS43_S_UP);
    step(&s, 0, 0, 0, 2910);

    /* Slow upward tracing becomes a drag at the timeout, rather than a swipe. */
    step(&s, 3, 0, -40, 3000);
    assert(!step(&s, 3, 0, -40, 3100).swipe);
    assert(!step(&s, 3, 0, -40, 3200).press);
    r = step(&s, 3, 0, -40, 3250);
    assert(r.press && r.move && !r.swipe);
    assert(step(&s, 3, 0, -500, 3260).move);

    /* Read errors or suspension release an active drag and require a full lift. */
    assert(tps43_gesture_cancel(&s));
    assert(!tps43_gesture_cancel(&s));
    r = step(&s, 3, 0, -500, 3300);
    assert(r.consume && !r.move && !r.swipe && !r.press);
    step(&s, 0, 0, 0, 3310);
    assert(step(&s, 3, 0, -500, 3400).swipe == TPS43_S_UP);
    step(&s, 0, 0, 0, 3410);

    /* Zero timeout disables the swipe arbitration and selects immediate drag. */
    r = tps43_gesture_update(&s, 3, 0, -500, 3500, 0, 150, 20);
    assert(r.press && r.move && !r.swipe);
    puts("three-finger gesture sequence tests passed");
    return 0;
}

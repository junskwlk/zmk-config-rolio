"""Exercise the actual delayed-click handler without relying on new sensor frames."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
s=(root/'modules/azoteq/drivers/input/tps43.c').read_text()
a=s.index('static void tps43_report_right_click(')
b=s.index('/**\n * @brief Main work handler',a)
functions=s[a:b]
stub=r"""
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "tps43_two_zoom.h"
#define K_FOREVER 0
#define K_MSEC(x) (x)
#define MAX(a,b) ((a)>(b)?(a):(b))
#define INPUT_BTN_1 257
#define CONTAINER_OF(ptr,type,member) ((type *)((char *)(ptr)-offsetof(type,member)))
struct k_work { int ready; };
struct k_work_delayable { struct k_work work; };
struct device { void *data; };
struct tps43_drv_data {
 const struct device *dev; int lock;
 struct k_work_delayable two_tap_work;
 struct tps43_two_zoom two_zoom; bool suspended;
};
static int tps43_work_q, clicks, reports;
static int64_t now,delay=-1;
static struct k_work_delayable *k_work_delayable_from_work(struct k_work *w) {
 return CONTAINER_OF(w,struct k_work_delayable,work);
}
static int64_t k_uptime_get(void) { return now; }
static void k_sem_take(int *lock,int timeout) { (void)timeout;assert(!*lock);*lock=1; }
static void k_sem_give(int *lock) { assert(*lock);*lock=0; }
static void k_work_reschedule_for_queue(int *queue,struct k_work_delayable *w,int64_t ms) {
 assert(queue==&tps43_work_q && w && ms>=0);delay=ms;
}
static void k_work_cancel_delayable(struct k_work_delayable *w) { assert(w);delay=-1; }
static void diag_report_key(const struct device *dev,int code,int value,bool sync,int timeout) {
 struct tps43_drv_data *d=dev->data;
 assert(d->lock && code==INPUT_BTN_1 && sync && timeout==K_FOREVER);
 assert(value==(reports%2==0));reports++;if(!value)clicks++;
}
"""
main=r"""
int main(void) {
 struct tps43_drv_data data={0};struct device dev={.data=&data};data.dev=&dev;
 tps43_two_zoom_update(&data.two_zoom,2,0,0,100);
 tps43_two_zoom_update(&data.two_zoom,0,0,0,150);
 now=150;tps43_schedule_two_tap(&dev);assert(delay==300 && clicks==0);
 now=449;tps43_two_tap_timeout(&data.two_tap_work.work);assert(clicks==0 && delay==1);
 now=450;tps43_two_tap_timeout(&data.two_tap_work.work);assert(clicks==1 && delay==-1);
 now=451;tps43_two_tap_timeout(&data.two_tap_work.work);assert(clicks==1);
 tps43_two_zoom_update(&data.two_zoom,2,0,0,1000);
 tps43_two_zoom_update(&data.two_zoom,0,0,0,1050);
 now=1100;tps43_two_zoom_update(&data.two_zoom,2,0,0,now);
 tps43_schedule_two_tap(&dev);assert(delay==-1);
 now=1400;tps43_two_tap_timeout(&data.two_tap_work.work);assert(clicks==1);
 tps43_two_zoom_update(&data.two_zoom,0,0,0,1500);
 tps43_two_zoom_update(&data.two_zoom,2,0,0,2000);
 tps43_two_zoom_update(&data.two_zoom,0,0,0,2050);
 now=2400;data.suspended=true;
 tps43_two_tap_timeout(&data.two_tap_work.work);
 assert(clicks==1 && delay==-1 && !data.two_zoom.pending_click);
}
"""
with tempfile.TemporaryDirectory(prefix='sofle-tap-timer-') as tmp:
 p=Path(tmp)/'test.c';binary=Path(tmp)/'test';p.write_text(stub+functions+main)
 subprocess.run(['clang','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
 '-I'+str(root/'modules/azoteq/drivers/input'),str(p),'-o',str(binary)],check=True)
 subprocess.run([str(binary)],check=True)
print('actual delayed click / dedicated queue / cancellation / suspend tests passed')

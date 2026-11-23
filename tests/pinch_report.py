"""Compile the actual pinch emitter and contact routing with sanitized stubs."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
s = (root / "modules/azoteq/drivers/input/tps43.c").read_text()
start = s.index("static void tps43_report_pinch(")
helper = s[start:s.index("\n}", start)+2]
m=s.index("static void tps43_set_zoom_modifier(")
modifier=s[m:s.index("\n}",m)+2]

a=s.index("    if (zoom.consume) {")
b=s.index("    if (two.consume)",a)
routing=s[a:b]

stub = r"""
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#define K_FOREVER 0
#define INPUT_BTN_SELECT 314
#define INPUT_BTN_0 256
#define INPUT_REL_HWHEEL 6
#define TPS43_ZOOM 4
#define CLAMP(v,l,h) ((v)<(l)?(l):((v)>(h)?(h):(v)))
struct tps43_drv_data { int32_t pinch_remainder; bool zoom_motion_logged, zoom_modifier_held; };
struct tps43_two_zoom_result { bool consume, move; };
#define LOG_INF(...) ((void)0)
struct tps43_config { int16_t zoom_sensitivity; bool zoom; };
struct device { struct tps43_drv_data *data; struct tps43_config *config; };
static int events[100][3], count;
static int diag_report_key(const struct device *d,int c,int v,bool sync,int t) {
 (void)d;(void)t;assert(sync);events[count][0]=1;events[count][1]=c;events[count++][2]=v; return 0;
}
static void diag_report_rel(const struct device *d,int c,int v,bool sync,int t) {
 (void)d;(void)t;assert(sync);events[count][0]=2;events[count][1]=c;events[count++][2]=v;
}
"""
routehead = r"""
static bool route(struct device *dev,bool consume,bool move,int16_t dx,int16_t dy,uint8_t flags) {
 struct tps43_drv_data *drv_data=dev->data;
 struct tps43_two_zoom_result zoom={.consume=consume,.move=move};
 bool is_drag_active=false;
 int16_t rel_x=dx,rel_y=dy;
 uint8_t gestures_events[2]={0,flags};
 (void)gestures_events;
"""
main = r"""
int main(void) {
 struct tps43_drv_data data={0}; struct tps43_config cfg={.zoom_sensitivity=10};
 struct device dev={&data,&cfg};
 tps43_set_zoom_modifier(&dev,true); assert(count==1 && data.zoom_modifier_held);
 assert(events[0][0]==1 && events[0][1]==INPUT_BTN_SELECT && events[0][2]==1);
 /* Command stays down across all frames, including tiny/zero movement. */
 for(int i=0;i<9;i++) assert(route(&dev,true,true,-1,0,0));
 assert(count==1 && data.pinch_remainder==90);
 assert(route(&dev,true,true,-1,0,0));assert(count==2 && data.zoom_motion_logged);
 assert(events[1][0]==2 && events[1][1]==INPUT_REL_HWHEEL && events[1][2]==1);
 assert(route(&dev,true,true,10,0,0));assert(count==3 && events[2][2]==-1);
 assert(route(&dev,true,true,-50,0,0));assert(count==4 && events[3][2]==5);
 assert(!route(&dev,false,true,-10,0,0));assert(count==4);
 assert(route(&dev,true,false,-10,0,0));assert(count==4);
 assert(route(&dev,true,true,0,20,0));assert(count==4);
 tps43_report_pinch(&dev,INT16_MIN);assert(count==5 && events[4][2]==127);
 tps43_report_pinch(&dev,INT16_MAX);assert(count==6 && events[5][2]==-127);
 /* Exactly one release, even if repeated cancellation paths run. */
 tps43_set_zoom_modifier(&dev,false);assert(count==7 && !data.zoom_modifier_held);
 assert(events[6][0]==1 && events[6][1]==INPUT_BTN_SELECT && events[6][2]==0);
 tps43_set_zoom_modifier(&dev,false);assert(count==7);
}
"""
with tempfile.TemporaryDirectory(prefix="sofle-pinch-test-") as tmp:
 p=Path(tmp)/"test.c"; binary=Path(tmp)/"test"
 p.write_text(stub+modifier+helper+routehead+routing+"\n return false;\n done: return true;\n}\n"+main)
 subprocess.run(["clang","-std=c11","-Wall","-Wextra","-Werror","-fsanitize=address,undefined",str(p),"-o",str(binary)],check=True)
 subprocess.run([str(binary)],check=True)
print("actual held-modifier zoom / sensitivity 10 / direction / fraction tests passed")

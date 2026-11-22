"""Compile the actual pinch emitter and contact routing with sanitized stubs."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
s = (root / "modules/azoteq/drivers/input/tps43.c").read_text()
start = s.index("static void tps43_report_pinch(")
helper = s[start:s.index("\n}", start)+2]
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
struct tps43_drv_data { bool pinch_contact; int32_t pinch_remainder; };
struct tps43_config { int16_t zoom_sensitivity; bool zoom; };
struct device { struct tps43_drv_data *data; struct tps43_config *config; };
static int events[100][3], count;
static void diag_report_key(const struct device *d,int c,int v,bool sync,int t) {
 (void)d;(void)t;assert(sync);events[count][0]=1;events[count][1]=c;events[count++][2]=v;
}
static void diag_report_rel(const struct device *d,int c,int v,bool sync,int t) {
 (void)d;(void)t;assert(sync);events[count][0]=2;events[count][1]=c;events[count++][2]=v;
}
"""
main = r"""
int main(void) {
 struct tps43_drv_data data={0}; struct tps43_config cfg={.zoom_sensitivity=50};
 struct device dev={&data,&cfg};
 /* Sensor X is screen Y: upward (-) zooms in (+), downward (+) out (-). */
 tps43_report_pinch(&dev,-1); assert(count==0 && data.pinch_remainder==50);
 tps43_report_pinch(&dev,-1); assert(count==3);
 assert(events[0][0]==1 && events[0][1]==INPUT_BTN_SELECT && events[0][2]==1);
 assert(events[1][0]==2 && events[1][1]==INPUT_REL_HWHEEL && events[1][2]==1);
 assert(events[2][0]==1 && events[2][1]==INPUT_BTN_SELECT && events[2][2]==0);
 tps43_report_pinch(&dev,2); assert(events[4][2]==-1 && events[5][2]==0);
 tps43_report_pinch(&dev,INT16_MIN); assert(events[7][2]==127 && events[8][2]==0);
 tps43_report_pinch(&dev,INT16_MAX); assert(events[10][2]==-127 && events[11][2]==0);
}
"""
with tempfile.TemporaryDirectory(prefix="sofle-pinch-test-") as tmp:
 p=Path(tmp)/"test.c"; binary=Path(tmp)/"test"
 p.write_text(stub+helper+main)
 subprocess.run(["clang","-std=c11","-Wall","-Wextra","-Werror","-fsanitize=address,undefined",str(p),"-o",str(binary)],check=True)
 subprocess.run([str(binary)],check=True)
print("actual hold-zoom emitter/order/direction/fraction tests passed")

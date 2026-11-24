"""Verify the actual one-finger emitter orders down/move/up for a drag."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
s=(root/'modules/azoteq/drivers/input/tps43.c').read_text()
a=s.index('static void tps43_report_one_finger(');b=s.index('\n}',a)+2
stub=r"""
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include "tps43_one_finger.h"
#define INPUT_BTN_0 256
#define INPUT_REL_X 0
#define INPUT_REL_Y 1
#define K_FOREVER 0
#define CLAMP(v,l,h) ((v)<(l)?(l):((v)>(h)?(h):(v)))
struct tps43_config { int sensitivity; };
struct device { struct tps43_config *config; };
static int events[20][3],count;
static void diag_report_key(const struct device *d,int c,int v,bool sync,int t) {
 (void)d;(void)t;assert(sync);events[count][0]=1;events[count][1]=c;events[count++][2]=v;
}
static void diag_report_rel(const struct device *d,int c,int v,bool sync,int t) {
 (void)d;(void)t;assert(sync==(c==INPUT_REL_Y));events[count][0]=2;events[count][1]=c;events[count++][2]=v;
}
"""
main=r"""
int main(void) {
 struct tps43_config cfg={.sensitivity=100};struct device dev={.config=&cfg};
 tps43_report_one_finger(&dev,(struct tps43_one_result){.press=true,.move_x=20,.move_y=-10,.release=true});
 assert(count==4);
 assert(events[0][0]==1 && events[0][1]==INPUT_BTN_0 && events[0][2]==1);
 assert(events[1][0]==2 && events[1][1]==INPUT_REL_X && events[1][2]==20);
 assert(events[2][0]==2 && events[2][1]==INPUT_REL_Y && events[2][2]==-10);
 assert(events[3][0]==1 && events[3][2]==0);
 tps43_report_one_finger(&dev,(struct tps43_one_result){.click=true});
 assert(count==6 && events[4][2]==1 && events[5][2]==0);
 tps43_report_one_finger(&dev,(struct tps43_one_result){.move_x=INT32_MAX,.move_y=INT32_MIN});
 assert(events[6][2]==INT16_MAX && events[7][2]==INT16_MIN);
}
"""
with tempfile.TemporaryDirectory(prefix='sofle-one-report-') as tmp:
 p=Path(tmp)/'test.c';binary=Path(tmp)/'test';p.write_text(stub+s[a:b]+main)
 subprocess.run(['clang','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
 '-I'+str(root/'modules/azoteq/drivers/input'),str(p),'-o',str(binary)],check=True)
 subprocess.run([str(binary)],check=True)
print('actual single-finger emitter down/move/up and range tests passed')

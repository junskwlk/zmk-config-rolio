"""Exercise the actual driver's shared scroll emitter with sensor-space values."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "modules/azoteq/drivers/input/tps43.c").read_text()
start = source.index("static void tps43_report_scroll(")
end = source.index("\n}", start) + 2
stub = r"""
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <assert.h>
#define INPUT_REL_HWHEEL 6
#define INPUT_REL_WHEEL 8
#define K_FOREVER 0
#define CLAMP(v, lo, hi) ((v)<(lo)?(lo):((v)>(hi)?(hi):(v)))
struct tps43_config { bool invert_scroll_x, invert_scroll_y; int scroll_sensitivity; };
struct device { const struct tps43_config *config; };
static int code, value, count;
static void diag_report_rel(const struct device *dev, int c, int v, bool sync, int timeout) {
    (void)dev; (void)timeout; assert(sync); code=c; value=v; count++;
}
"""
main = r"""
int main(void) {
    struct tps43_config cfg = {.invert_scroll_y=true, .scroll_sensitivity=30};
    struct device dev = {.config=&cfg};
    /* The existing listener swaps HWHEEL/WHEEL after this emitter. */
    tps43_report_scroll(&dev,50,0); assert(code==INPUT_REL_HWHEEL && value==15);
    tps43_report_scroll(&dev,0,-50); assert(code==INPUT_REL_WHEEL && value==15);
    tps43_report_scroll(&dev,0,50); assert(code==INPUT_REL_WHEEL && value==-15);
    tps43_report_scroll(&dev,10,50); assert(code==INPUT_REL_WHEEL && value==-15);
    tps43_report_scroll(&dev,INT32_MIN,0);
    assert(code==INPUT_REL_HWHEEL && value==INT16_MIN);
    tps43_report_scroll(&dev,0,INT32_MIN);
    assert(code==INPUT_REL_WHEEL && value==INT16_MAX);
    cfg.invert_scroll_x=true;
    tps43_report_scroll(&dev,INT32_MIN,0); assert(value==INT16_MAX);
    assert(count==7);
}
"""
with tempfile.TemporaryDirectory(prefix="sofle-scroll-test-") as tmp:
    path = Path(tmp) / "test.c"
    binary = Path(tmp) / "test"
    path.write_text(stub + source[start:end] + main)
    subprocess.run(["clang", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-fsanitize=address,undefined", str(path), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print("shared scroll emitter transform/range tests passed")

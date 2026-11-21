"""Verify the actual driver emits address + data + STOP via one I2C write."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
s = (root / "modules/azoteq/drivers/input/tps43.c").read_text()
a = s.index("static void tps43_end_communication_window(")
b = s.index("\n}", a) + 2
fn = s[a:b]
stub = r"""
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <errno.h>
#define TPS43_REG_END_COMM_WINDOW 0xEEEE
#define LOG_DBG(...) ((void)0)
struct tps43_config { int i2c_bus; };
struct device { const struct tps43_config *config; };
static int calls;
static void sys_put_be16(uint16_t val, uint8_t *out) {
    out[0] = val >> 8; out[1] = val;
}
static int i2c_write_dt(const int *bus, const uint8_t *buf, size_t len) {
    assert(*bus == 7);
    assert(len == 3 && buf[0] == 0xEE && buf[1] == 0xEE && buf[2] == 0);
    calls++; return 0;
}
"""
main = r"""
int main(void) {
    struct tps43_config cfg = {.i2c_bus = 7};
    struct device dev = {.config = &cfg};
    tps43_end_communication_window(&dev);
    assert(calls == 1);
}
"""
with tempfile.TemporaryDirectory(prefix="sofle-protocol-test-") as tmp:
    p = Path(tmp) / "test.c"
    binary = Path(tmp) / "test"
    p.write_text(stub + fn + main)
    subprocess.run(["clang", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-fsanitize=address,undefined", str(p), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print("communication end command test passed")

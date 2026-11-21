"""Run the actual driver's init function with an interrupt injected while arming RDY."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
driver = (root / "modules/azoteq/drivers/input/tps43.c").read_text()
start = driver.index("static int tps43_init(")
end = driver.index("#define TPS43_INIT(inst)", start)
init_function = driver[start:end]
stubs = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <errno.h>
#include <stdio.h>
#define LOG_INF(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define BIT(n) (1u << (n))
#define K_MSEC(ms) (ms)
#define GPIO_OUTPUT_INACTIVE 1
#define GPIO_INPUT 2
#define GPIO_INT_EDGE_TO_ACTIVE 3
struct device { const char *name; void *data; const void *config; };
struct gpio { const void *port; unsigned pin; };
struct k_sem { bool ready; };
struct k_work { bool ready; };
struct gpio_callback { bool ready; };
struct tps43_drv_data {
    const struct device *dev;
    struct k_sem lock;
    struct k_work work;
    struct gpio_callback rdy_cb;
    bool initialized, suspended;
};
struct tps43_config {
    struct { const struct device *bus; unsigned addr; } i2c_bus;
    struct gpio rst_gpio, rdy_gpio;
};
static const struct device *diag_dev;
static int diag_online;
static void atomic_set(int *target, int value) { *target = value; }
static struct tps43_drv_data *active;
static bool armed, callback_added, ready_high, inject_irq;
static int submitted;
static bool device_is_ready(const struct device *dev) { return dev != NULL; }
static void k_sem_init(struct k_sem *s, int initial, int limit) {
    assert(initial == 1 && limit == 1); s->ready = true;
}
static void tps43_work_handler(void) {}
static void k_work_init(struct k_work *w, void (*handler)(void)) {
    assert(handler != NULL); w->ready = true;
}
static int k_work_submit(struct k_work *w) {
    assert(w->ready && active->lock.ready && active->initialized);
    submitted++; return 0;
}
static void tps43_rdy_callback(void) { k_work_submit(&active->work); }
static int tps43_reset_values(const struct device *dev) {
    (void)dev; active->initialized = false; return 0;
}
static int gpio_pin_configure_dt(const struct gpio *g, int flags) {
    (void)g; (void)flags; return 0;
}
static int gpio_pin_set_dt(const struct gpio *g, int value) {
    (void)g; (void)value; return 0;
}
static void k_sleep(int ms) { (void)ms; }
static int check_reset_and_reconfigure(const struct device *dev) {
    (void)dev; assert(!armed); return 0;
}
static void tps43_dump_registers(const struct device *dev) {
    (void)dev; assert(!armed);
}
static void gpio_init_callback(struct gpio_callback *cb, void (*fn)(void), unsigned mask) {
    assert(fn != NULL && mask != 0); cb->ready = true;
}
static int gpio_add_callback(const void *port, struct gpio_callback *cb) {
    (void)port; assert(cb->ready); callback_added = true; return 0;
}
static int gpio_pin_interrupt_configure_dt(const struct gpio *g, int flags) {
    (void)g; assert(flags == GPIO_INT_EDGE_TO_ACTIVE);
    assert(callback_added && active->lock.ready && active->work.ready);
    assert(active->initialized);
    armed = true;
    if (inject_irq) { tps43_rdy_callback(); }
    return 0;
}
static int gpio_pin_get_dt(const struct gpio *g) { (void)g; return ready_high; }
'''
main = r'''
int main(void) {
    struct device bus = {.name = "i2c"};
    struct tps43_config cfg = {
        .i2c_bus = {.bus = &bus, .addr = 0x74},
        .rst_gpio = {.port = &bus, .pin = 0},
        .rdy_gpio = {.port = &bus, .pin = 2},
    };
    for (int scenario = 0; scenario < 3; scenario++) {
        struct tps43_drv_data data = {0};
        struct device dev = {.name = "trackpad", .data = &data, .config = &cfg};
        active = &data;
        armed = callback_added = false;
        ready_high = scenario == 1;
        inject_irq = scenario == 2;
        submitted = 0;
        assert(tps43_init(&dev) == 0);
        assert(armed && data.initialized);
        assert(diag_dev == &dev && diag_online == 1);
        assert(submitted == (scenario == 0 ? 0 : 1));
    }
    puts("driver init interrupt/rdy tests passed");
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix="sofle-init-test-") as tmp:
    source = Path(tmp) / "init_test.c"
    binary = Path(tmp) / "init_test"
    source.write_text(stubs + init_function + main)
    subprocess.run(["clang", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-fsanitize=address,undefined", str(source), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)

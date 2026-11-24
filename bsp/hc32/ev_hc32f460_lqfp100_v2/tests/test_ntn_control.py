"""Host regressions for the production NTN control functions (Python + GCC)."""
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

BSP = Path(__file__).resolve().parents[1]


def function(source, name):
    match = re.search(r"(?m)^(?:static\s+)?[\w*]+\s+" + name +
                      r"\([^;{]*\)\s*\{", source)
    if not match:
        raise AssertionError(f"Missing production function: {name}")
    depth = 1
    pos = match.end()
    while depth:
        depth += (source[pos] == "{") - (source[pos] == "}")
        pos += 1
    return source[match.start():pos]


NTN_FUNCTIONS = [
    "ntn_command_receive", "ntn_debug_lock_release", "ntn_debug_lock_is_held",
    "ntn_prepare_pc_uart", "at_line_dispatch", "ntn_rx_thread_entry",
    "at_line_reset_locked", "at_line_timeout_entry",
    "at_line_timeout_process", "ntn_exec_cmd_locked", "ntn_command_recover_locked",
    "ntn_exec_cmd_internal", "ntn_exec_cmd", "ntn_debug_exec_cmd", "ntn_at",
]
POWER_FUNCTIONS = ["hw_power_ntn_set", "hw_ntn_wakeup_set", "hw_ntn_wakeup_pulse", "power_wakeup_ntn"]

STUBS = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
typedef int rt_bool_t;
typedef int rt_err_t;
typedef size_t rt_size_t;
typedef uint8_t rt_uint8_t;
typedef uint16_t rt_uint16_t;
typedef uint32_t rt_uint32_t;
typedef int32_t rt_int32_t;
typedef uint32_t rt_tick_t;
static int _identity_query_active;
#define RT_TRUE 1
#define RT_FALSE 0
#define RT_NULL NULL
#define RT_EOK 0
#define RT_ERROR 1
#define RT_ETIMEOUT 2
#define RT_EFULL 3
#define RT_EBUSY 4
#define RT_EINVAL 5
#define RT_WAITING_FOREVER -1
#define RT_UNUSED(x) (void)(x)
#define LOG_I(...) ((void)0)
#define LOG_W(...) ((void)0)
#define rt_strcmp strcmp
#define rt_strncmp strncmp
#define rt_strlen strlen
#define rt_memcpy memcpy
#define rt_kprintf printf
#define rt_snprintf snprintf
#define NTN_RESP_BUF_MAX_SZ 1536
#define NTN_AT_INTERBYTE_MS 2000U
#define NTN_BOOT_PIN 0
#define NTN_WAKEUP_PIN 1
#define NTN_RST_PIN 2
#define NTN_PWR_EN_PIN 3
#define PA_PWREN_PIN 4
#define PIN_LOW 0
#define PIN_HIGH 1
#define PIN_MODE_OUTPUT 1
#define PARAM_MODE_AT_PASSTHROUGH 0
static int _mutex_cmd, _mutex_resp, _mutex_at_line;
struct semaphore { int count; };
static struct semaphore _sem_resp, _sem_rx, _sem_awake;
static struct { unsigned at_cmds_sent, at_cmds_ok, at_cmds_err, rx_bytes, bridge_ntn_to_pc; } _stats;
static rt_bool_t _inited = RT_TRUE, _cmd_waiting, _cmd_final_received;
static rt_uint32_t _ntn_boot_count;
static rt_bool_t _cmd_resp_truncated, _cmd_expect_reboot, _result_line_overflow;
static rt_bool_t _cmd_recovery_required, _pc_command_active, _at_discarding;
static rt_bool_t _at_timeout_pending, _ntn_power_state;
static rt_bool_t _modem_sleeping, _debug_locked, _bridge_active = 1;
static rt_tick_t _debug_lock_deadline, _pc_command_deadline;
static void (*_raw_rx_handler)(const rt_uint8_t *, rt_size_t, rt_bool_t);
static void *_ntn_dev = (void *)1;
static char *_cur_resp_buf, _result_line[64];
static rt_size_t _cur_resp_sz, _cur_resp_idx, _result_line_len, _at_line_len;
static rt_tick_t _at_line_start_tick, _at_last_byte_tick, now, late_at;
static rt_err_t _cmd_status;
static int isr, reply, sent_commands, late_pending, pc_writes, reset_writes;
static int pins[5], writes[16], write_count;
static unsigned write_times[16];
static int wake_response, reject_cmd_lock, rx_test;
static char uart_tx[2048], pc_rx[2048], last_pc_error[128], _local_response[2048];
static size_t uart_tx_len, pc_rx_len, rx_pos;
static const char *rx_source;
static jmp_buf rx_done;
/* Production action types are inserted before this stub block. */
static int cpu_resets, rtt_reset;
static char s_command[384], s_local_response[2048];
#define CTRL2_TYPE_LOCAL_FINAL 1
static rt_bool_t handle_reboot(const char *, char *, rt_size_t, mcu_cmd_action_t *);
void mcu_local_cmd_apply_reboot(const mcu_cmd_action_t *);
static int mcu_local_cmd_execute(const rt_uint8_t *s, rt_size_t n, char *r,
                                rt_size_t size, mcu_cmd_action_t *action) {
    char command[384];
    if (n >= sizeof(command)) return 0;
    memcpy(command, s, n); command[n] = 0;
    action->type = MCU_CMD_ACTION_NONE;
    return handle_reboot(command, r, size, action);
}
static void mcu_local_cmd_complete_baud(rt_uint32_t baud, rt_bool_t ready, char *out, rt_size_t size)
{ (void)baud; (void)ready; (void)out; (void)size; }
static void rt_hw_cpu_reset(void) {
    assert(!isr && now == 500);
    if (!rtt_reset) {
        assert(pc_writes == 1);
        assert(strstr(last_pc_error, "REBOOTING") && strstr(last_pc_error, "OK"));
    }
    cpu_resets++;
}
static void local_action_apply(const mcu_cmd_action_t *action) { (void)action; }
static void at_line_timeout_process(void);
void hw_ntn_wakeup_set(rt_bool_t);
void hw_ntn_wakeup_pulse(rt_uint32_t, rt_bool_t);
static void ntn_command_receive(const rt_uint8_t *, rt_size_t);
static void feed(const char *s) { ntn_command_receive((const rt_uint8_t *)s, strlen(s)); }
static rt_tick_t rt_tick_get(void) { return now; }
static rt_tick_t rt_tick_from_millisecond(rt_uint32_t ms) { return ms; }
static int rt_mutex_take(void *m, int timeout) {
    (void)timeout; assert(!isr);
    return (m == &_mutex_cmd && reject_cmd_lock) ? -RT_EBUSY : 0;
}
static void rt_mutex_release(void *m) { (void)m; assert(!isr); }
static void rt_sem_release(struct semaphore *s) { s->count++; }
static int rt_sem_trytake(struct semaphore *s) {
    if (!s->count) return -RT_ETIMEOUT;
    s->count--; return 0;
}
static int rt_sem_take(struct semaphore *s, rt_tick_t timeout) {
    if (rx_test && s == &_sem_rx) longjmp(rx_done, 1);
    if (s == &_sem_awake) {
        if (wake_response) { now += 700; feed("\r\nEXIT DEEPSLEEP\r\n"); }
        if (s->count) { s->count--; return 0; }
        now += timeout; return -RT_ETIMEOUT;
    }
    if (reply == 1) feed("\r\nOK\r\n");
    if (reply == 2) feed("\r\nBOOK\r\n");
    if (reply == 3) feed("\r\n+CME ERROR: 10\r\n");
    if (reply == 4) feed("\r\nREBOOTING\r\n");
    if (reply == 5) feed("\r\nManufacturer:Mlink-tech Corporation\r\nModel:MS150\r\nVersion:V00_R005_CA033\r\nIMEI:862130042600001\r\nAdditional modem information\r\nOK\r\n");
    if (s->count) { s->count--; return 0; }
    now += timeout; return -RT_ETIMEOUT;
}
static void rt_thread_mdelay(unsigned ms) {
    assert(!isr); now += ms;
    if (late_pending && now >= late_at) { late_pending = 0; feed("\r\nOK\r\n"); }
}
static rt_size_t ntn_uart_write(const void *data, rt_size_t len) {
    assert(uart_tx_len + len < sizeof(uart_tx));
    memcpy(uart_tx + uart_tx_len, data, len); uart_tx_len += len;
    uart_tx[uart_tx_len] = 0;
    if (len > 1) sent_commands++;
    return len;
}
static void pc_rs232_send_str(const char *s) {
    assert(!isr); pc_writes++;
    snprintf(last_pc_error, sizeof(last_pc_error), "%s", s);
}
static void ctrl2_send_text(rt_uint8_t type, rt_uint16_t request_id, const char *text) {
    assert(type == CTRL2_TYPE_LOCAL_FINAL && request_id == 1);
    pc_rs232_send_str(text); /* Capture the final response before CPU reset. */
}
static void ctrl2_apply_baud(rt_uint16_t id, const mcu_cmd_action_t *action) {
    (void)id; (void)action; assert(0);
}
static void pc_rs232_send(const void *data, rt_size_t len) {
    assert(pc_rx_len + len < sizeof(pc_rx));
    memcpy(pc_rx + pc_rx_len, data, len); pc_rx_len += len; pc_rx[pc_rx_len] = 0;
}
static rt_size_t rt_device_read(void *dev, int pos, void *data, rt_size_t count) {
    size_t remaining = strlen(rx_source) - rx_pos;
    (void)dev; (void)pos;
    if (remaining > count) remaining = count;
    memcpy(data, rx_source + rx_pos, remaining); rx_pos += remaining;
    return remaining;
}
static void ntn_rx_line_feed(rt_uint8_t ch) { (void)ch; }
static int param_get_work_mode(void) { return PARAM_MODE_AT_PASSTHROUGH; }
static void task_wdt_heartbeat_by_name(const char *name) { (void)name; }
static void rt_pin_mode(int pin, int mode) { (void)pin; (void)mode; assert(!isr); }
static void rt_pin_write(int pin, int level) {
    pins[pin] = level;
    if (pin == NTN_RST_PIN) reset_writes++;
    if (pin == NTN_WAKEUP_PIN) {
        assert(write_count < 16);
        writes[write_count] = level; write_times[write_count++] = now;
    }
}
static int rt_pin_read(int pin) { return pins[pin]; }
static void reset_state(void) {
    _cmd_waiting = _cmd_final_received = _cmd_resp_truncated = _cmd_expect_reboot = 0;
    _cmd_recovery_required = _pc_command_active = _result_line_overflow = 0;
    _cur_resp_buf = NULL; _cur_resp_sz = _cur_resp_idx = _result_line_len = 0;
    _sem_resp.count = _sem_rx.count = 0;
    _sem_awake.count = _modem_sleeping = _debug_locked = 0;
    _pc_command_deadline = _debug_lock_deadline = 0;
    now = reply = sent_commands = late_pending = pc_writes = 0;
    write_count = reset_writes = 0;
    cpu_resets = rtt_reset = 0;
    wake_response = reject_cmd_lock = rx_test = 0;
    uart_tx_len = pc_rx_len = rx_pos = 0;
    uart_tx[0] = pc_rx[0] = last_pc_error[0] = 0;
}
'''

CASES = r'''
int main(void) {
    char response[80];
    /* Both PC modes acknowledge before delay/reset; nothing reaches MS150. */
    reset_state();
    at_line_dispatch((const rt_uint8_t *)"AT+MCUREBOOT", 12);
    assert(cpu_resets == 1 && now == 500 && sent_commands == 0);
    reset_state();
    maint_local_command(NULL, (const uint8_t *)"AT+MCUREBOOT", 12);
    assert(cpu_resets == 1 && now == 500 && sent_commands == 0);
    reset_state();
    ctrl2_handle_local(1, (const rt_uint8_t *)"AT+MCUREBOOT\r\n", 14);
    assert(cpu_resets == 1 && now == 500 && sent_commands == 0);
    reset_state();
    ctrl2_handle_local(1, (const rt_uint8_t *)"AT+MCUREBOOT\r", 13);
    assert(cpu_resets == 0 && strstr(last_pc_error, "MCUERROR:1"));
    reset_state(); rtt_reset = 1;
    assert(sys_reboot(2, NULL) == -RT_EINVAL && cpu_resets == 0 && now == 0);
    assert(sys_reboot(1, NULL) == RT_EOK && cpu_resets == 1 && now == 500);
    reset_state(); {
        mcu_cmd_action_t action = { .type = MCU_CMD_ACTION_NONE };
        assert(!handle_reboot("AT+MCUREBOOT?", response, sizeof(response), &action));
        assert(!handle_reboot("AT+MCUREBOOT=1", response, sizeof(response), &action));
        mcu_local_cmd_apply_reboot(&action);
        mcu_local_cmd_apply_reboot(NULL);
        assert(cpu_resets == 0 && now == 0);
    }
    reset_state(); reply = 1;
    assert(ntn_exec_cmd("AT", response, sizeof(response), 3000) == RT_EOK);
    assert(strstr(response, "OK") && sent_commands == 1);

    /* Automatic and Bluetooth AT clients must wake a sleeping modem before TX. */
    reset_state(); feed("\r\nENTER DEEPSLEEP\r\n"); wake_response = 1; reply = 1;
    assert(ntn_exec_cmd("ATI", response, sizeof(response), 3000) == RT_EOK);
    if (_modem_sleeping || sent_commands != 1 || pins[NTN_WAKEUP_PIN] != PIN_LOW) { fprintf(stderr, "FAIL: automatic client did not wake modem before TX\n"); return 42; }
    reset_state(); feed("\r\nENTER DEEPSLEEP\r\n"); wake_response = 1; reply = 1; _debug_locked = 1; _debug_lock_deadline = 60000;
    assert(ntn_exec_cmd_internal("ATI", response, sizeof(response), 3000, RT_TRUE) == RT_EOK);
    if (_modem_sleeping || sent_commands != 1 || strcmp(uart_tx, "ATI\r")) { fprintf(stderr, "FAIL: debug client did not wake modem before TX\n"); return 42; }
    /* Unknown-result protection still forbids a new wake/probe/command. */
    reset_state(); _cmd_recovery_required = 1; _modem_sleeping = 1; _debug_locked = 1; _debug_lock_deadline = 60000;
    assert(ntn_exec_cmd_internal("ATI", response, sizeof(response), 3000, RT_TRUE) == -RT_ETIMEOUT);
    assert(sent_commands == 0 && write_count == 0);

    /* Streaming debug callers must preserve a long response's terminal OK. */
    reset_state(); reply = 5; _debug_locked = 1; _debug_lock_deadline = 60000;
    assert(ntn_debug_exec_cmd("ATI", NULL, 0, 3000) == RT_EOK);
    assert(_cmd_final_received && !_cmd_resp_truncated);

    /* A payload ending in OK is not a terminal result. */
    reset_state(); reply = 2;
    assert(ntn_exec_cmd("ATI", response, sizeof(response), 3000) == -RT_ETIMEOUT);
    assert(_cmd_recovery_required);
    /* No new command is put on the wire while the previous result is unknown. */
    reply = 1; strcpy(response, "stale banner");
    assert(ntn_exec_cmd("ATI", response, sizeof(response), 3000) == -RT_ETIMEOUT);
    assert(response[0] == 0 && sent_commands == 1);

    /* A delayed final result drains the old transaction before the next TX. */
    late_pending = 1; late_at = now + 400;
    assert(ntn_exec_cmd("ATI", response, sizeof(response), 3000) == RT_EOK);
    assert(!late_pending && sent_commands == 2 && !_cmd_recovery_required);

    reset_state(); reply = 3;
    assert(ntn_exec_cmd("AT", response, sizeof(response), 3000) == -RT_ERROR);
    assert(!_cmd_recovery_required);
    reset_state(); reply = 1;
    assert(ntn_exec_cmd("AT", NULL, 0, 3000) == RT_EOK);
    assert(ntn_exec_cmd("AT", response, 2, 3000) == -RT_EFULL);
    assert(!_cmd_recovery_required);

    /* Terminal lines may be split across arbitrary UART chunks. */
    reset_state(); _cmd_waiting = 1; _cur_resp_buf = response; _cur_resp_sz = sizeof(response);
    feed("\r\nO"); assert(!_cmd_final_received);
    feed("K\r\n"); assert(_cmd_final_received && _cmd_status == RT_EOK);
    reset_state(); feed("EXIT DEEPSLEEP\r\n"); assert(!_cmd_final_received);
    reset_state(); _cmd_recovery_required = 1;
    feed("RDY\r\n");
    assert(!_cmd_final_received && !_cmd_recovery_required && _ntn_boot_count == 1U);
    reset_state(); _cmd_waiting = 1;
    feed("RDY\r\n");
    assert(!_cmd_final_received && _ntn_boot_count == 2U);
    reset_state(); reply = 4;
    assert(ntn_exec_cmd("AT+MKRB", response, sizeof(response), 3000) == RT_EOK);

    /* Side-effecting shell commands must not be transmitted twice on timeout. */
    reset_state(); { char *args[] = {"ntn_at", "AT+MKUDPSEND=1,01,0"}; ntn_at(2, args); }
    assert(sent_commands == 1);
    reset_state(); _pc_command_active = 1; _pc_command_deadline = 60000;
    assert(ntn_exec_cmd("ATI", response, sizeof(response), 3000) == -RT_EBUSY);
    assert(sent_commands == 0);

    /* PC manual retries are not blocked by its own unfinished transaction. */
    reset_state();
    at_line_dispatch((const rt_uint8_t *)"AT+CGMI", 7);
    assert(_pc_command_active && sent_commands == 1 && pc_writes == 0);
    at_line_dispatch((const rt_uint8_t *)"AT+CGMI", 7);
    assert(sent_commands == 2 && pc_writes == 0);
    assert(!strcmp(uart_tx, "AT+CGMI\rAT+CGMI\r"));
    now = 60001;
    assert(!ntn_debug_lock_is_held() && !_cmd_recovery_required);
    at_line_dispatch((const rt_uint8_t *)"AT+CGMI", 7);
    assert(sent_commands == 3 && pc_writes == 0);

    /* Reproduce ENTER DEEPSLEEP -> PC TX -> EXIT -> PC manual retry. */
    reset_state(); feed("\r\nENTER DEEPSLEEP\r\n");
    assert(_modem_sleeping);
    wake_response = 1;
    at_line_dispatch((const rt_uint8_t *)"AT+CGMI", 7);
    assert(!_modem_sleeping && now >= 700 && pins[NTN_WAKEUP_PIN] == PIN_LOW);
    assert(sent_commands == 1 && pc_writes == 0 && !strcmp(uart_tx, "AT+CGMI\r"));
    feed("\r\nEXIT DEEPSLEEP\r\n");
    at_line_dispatch((const rt_uint8_t *)"AT+CGMI", 7);
    assert(sent_commands == 2 && pc_writes == 0);

    /* Missing ready URC does not permanently reject the user's next attempt. */
    reset_state(); feed("\r\nENTER DEEPSLEEP\r\n");
    at_line_dispatch((const rt_uint8_t *)"AT+CGMI", 7);
    assert(sent_commands == 1 && pc_writes == 0);
    feed("\r\nRDY\r\n"); assert(!_pc_command_active && !_modem_sleeping);

    /* Genuine other-client ownership still returns BUSY to PC. */
    reset_state(); reject_cmd_lock = 1;
    at_line_dispatch((const rt_uint8_t *)"AT+CGMI", 7);
    assert(sent_commands == 0 && strstr(last_pc_error, "+MCUERROR:6"));

    /* Exercise the actual RX loop: modem URCs and responses stay byte-exact on PC. */
    reset_state(); rx_source = "\r\nCopyright (c) 2025 MLINK.\r\nRDY\r\nENTER DEEPSLEEP\r\nEXIT DEEPSLEEP\r\nAT+CGMI\r\nMlink-tech Corporation\r\nOK\r\n";
    rx_test = 1;
    if (setjmp(rx_done) == 0) ntn_rx_thread_entry(NULL);
    rx_test = 0;
    assert(pc_rx_len == strlen(rx_source) && !memcmp(pc_rx, rx_source, pc_rx_len));

    /* Hard timer callback must never take a mutex or write to a UART. */
    reset_state(); _at_line_len = 3; _at_last_byte_tick = 0; now = 2000;
    isr = 1; at_line_timeout_entry(NULL); isr = 0;
    assert(_at_timeout_pending && _sem_rx.count == 1 && pc_writes == 0);
    at_line_timeout_process(); assert(_at_line_len == 0 && pc_writes == 1);
    _at_line_len = 4; _at_last_byte_tick = now; _at_timeout_pending = 1;
    at_line_timeout_process(); assert(_at_line_len == 4 && pc_writes == 1);

    reset_state(); pins[NTN_WAKEUP_PIN] = PIN_LOW;
    hw_ntn_wakeup_pulse(200, RT_TRUE);
    assert(write_count == 3 && writes[0] == 1 && writes[1] == 0 && writes[2] == 1);
    assert(write_times[1] - write_times[0] == 10 && write_times[2] - write_times[1] == 200);
    reset_state(); pins[NTN_WAKEUP_PIN] = PIN_HIGH;
    hw_ntn_wakeup_pulse(200, RT_FALSE);
    assert(write_count == 3 && writes[0] == 0 && writes[1] == 1 && writes[2] == 0);
    reset_state(); _ntn_power_state = 1;
    hw_power_ntn_set(1); assert(reset_writes == 0 && pins[NTN_WAKEUP_PIN] == 0);
    reset_state(); { char *args[] = {"power_wakeup_ntn", "pulse_low", "-1"};
        assert(power_wakeup_ntn(3, args) == -RT_EINVAL && write_count == 0); }
    puts("NTN control regressions passed");
    return 0;
}
'''


def main():
    compiler = os.environ.get("HOST_CC") or shutil.which("gcc")
    if not compiler:
        raise SystemExit("Set HOST_CC to the host GCC executable")
    ntn = (BSP / "applications/ntn_ms150.c").read_text(encoding="utf-8")
    power = (BSP / "applications/hw_power.c").read_text(encoding="utf-8")
    local = (BSP / "applications/mcu_local_cmd.c").read_text(encoding="utf-8")
    router = (BSP / "applications/app_telemetry.c").read_text(encoding="utf-8")
    ble = (BSP / "applications/ble_uart.c").read_text(encoding="utf-8")
    main_source = (BSP / "applications/main.c").read_text(encoding="utf-8")
    header = (BSP / "applications/mcu_local_cmd.h").read_text(encoding="utf-8")
    action_types = header[header.index("typedef enum"):header.index("/*\n * Execute")]
    code = "#include <stdint.h>\ntypedef uint32_t rt_uint32_t;\n" + action_types + STUBS
    code += function(local, "handle_reboot") + function(local, "mcu_local_cmd_apply_reboot")
    code += function(router, "maint_local_command")
    code += "\n".join(function(ble, n) for n in (
        "ctrl2_trim_line", "ctrl2_is_printable", "ctrl2_handle_local"))
    code += function(main_source, "sys_reboot")
    code += "\n".join(function(ntn, n) for n in NTN_FUNCTIONS)
    code += "\n" + "\n".join(function(power, n) for n in POWER_FUNCTIONS) + CASES
    with tempfile.TemporaryDirectory(prefix="ntn_control_") as directory:
        source = Path(directory) / "test.c"
        binary = Path(directory) / ("test.exe" if os.name == "nt" else "test")
        source.write_text(code, encoding="utf-8")
        subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                        str(source), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()

"""Host regressions for production Bluetooth UART diagnostics and CTRL2 RX."""
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
from test_ntn_control import function

BSP = Path(__file__).resolve().parents[1]
FUNCTIONS = (
    "ctrl2_crc16", "ble_write_all_locked", "ble_data_write_locked", "ble_uart_send", "ble_ctrl2_send",
    "ctrl2_drop_prefix", "ctrl2_parse_stream", "ble_test_dump_rx",
    "ble_rx_process_chunk", "bt_test", "ble_test_tx_lock", "bt_send",
    "ble_test_hex_nibble", "bt_hex",
)
STUBS = r'''
#include "ntn_link.h"
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t rt_uint8_t;
typedef uint16_t rt_uint16_t;
typedef uint32_t rt_uint32_t;
typedef uint32_t rt_tick_t;
typedef size_t rt_size_t;
typedef int rt_bool_t;
typedef int rt_base_t;
#define RT_NULL NULL
#define RT_TRUE 1
#define RT_FALSE 0
#define RT_EOK 0
#define RT_ERROR 1
#define RT_EBUSY 7
#define RT_EINVAL 10
#define RT_WAITING_FOREVER -1
#define rt_strlen strlen
#define rt_strcmp strcmp
#define rt_memcpy memcpy
#define rt_memmove memmove
static void *s_ble_dev = (void *)1;
static int s_tx_lock, locked, irq, power_on = 1, stall_write, dispatches;
static uint32_t now;
static volatile rt_uint32_t s_uart_flags;
static int s_ntn_pending, s_ntn_request_active;
static rt_uint8_t s_rx_stream[CTRL2_RX_BUFFER_SIZE], s_tx_frame[CTRL2_MAX_FRAME_SIZE];
static rt_size_t s_rx_length;
static rt_tick_t s_rx_partial_tick;
static unsigned s_protocol_errors, s_crc_errors, s_rx_frames;
static unsigned outbound_errors;
static unsigned char tx[4096];
static size_t tx_len;
static char logs[16384];
static size_t log_len;
static int bt_test(int, char **);
static int rt_kprintf(const char *fmt, ...) {
    int n; va_list ap;
    va_start(ap, fmt);
    n = vsnprintf(logs + log_len, sizeof(logs) - log_len, fmt, ap);
    va_end(ap); assert(n >= 0 && (size_t)n < sizeof(logs) - log_len);
    log_len += (size_t)n; return n;
}
static int rt_mutex_take(void *mutex, int timeout) {
    (void)mutex; (void)timeout; assert(!irq && !locked); locked = 1; return 0;
}
static void rt_mutex_release(void *mutex) { (void)mutex; assert(locked && !irq); locked = 0; }
static rt_base_t rt_hw_interrupt_disable(void) { int old = irq; irq = 1; return old; }
static void rt_hw_interrupt_enable(rt_base_t old) { irq = old; }
static rt_tick_t rt_tick_get(void) { return now; }
static rt_tick_t rt_tick_from_millisecond(unsigned ms) { return ms; }
static void rt_thread_mdelay(unsigned ms) { assert(!irq); now += ms; }
static int hw_power_bt_get(void) { return power_on; }
static ntnl_decoder host_wire_decoder;
static void host_wire_data(unsigned char kind,unsigned short id,const unsigned char *p,unsigned short n) {
    assert(kind==NTNL_DATA && id==0 && tx_len+n<=sizeof(tx));
    memcpy(tx+tx_len,p,n);tx_len+=n;
}
static rt_size_t rt_device_write(void *dev, int pos, const void *data, size_t len) {
    (void)dev; (void)pos; assert(locked && !irq);
    if (stall_write) return 0;
    if (len > 2) len = 2; /* Exercise partial writes. */
    ntnl_feed(&host_wire_decoder,data,(unsigned)len,host_wire_data);return len;
}
static void ctrl2_handle_frame(const rt_uint8_t *frame, rt_uint16_t len) {
    char *args[] = {"bt_test", "on"};
    (void)frame; (void)len; dispatches++;
    assert(s_uart_flags & BLE_UART_RX_BUSY);
    assert(bt_test(2, args) == -RT_EBUSY); /* Do not switch inside a parser callback. */
}
static void ctrl2_handle_local(rt_uint16_t id, const rt_uint8_t *p, rt_uint16_t n) {
    (void)id; (void)p; (void)n; assert(0);
}
static void ctrl2_handle_ntn(rt_uint16_t id, const rt_uint8_t *p, rt_uint16_t n) {
    (void)id; (void)p; (void)n; assert(0);
}
static void ctrl2_send_text(rt_uint8_t type, rt_uint16_t id, const char *text) {
    (void)id; assert(type == CTRL2_TYPE_LOCAL_FINAL && strstr(text, "MCUERROR:18"));
    outbound_errors++;
}
static void clear_io(void) { tx_len = log_len = 0; logs[0] = 0; }
'''
CASES = r'''
int main(void) {
    char *on[] = {"bt_test", "on"}, *off[] = {"bt_test", "off"};
    char *text[] = {"bt_send", "PING"}, *cr[] = {"bt_send", "PING", "cr"};
    char *none[] = {"bt_send", "PING", "none"};
    char *hex[] = {"bt_hex", "0055aA0d0A"}, *bad[] = {"bt_hex", "00GG"};
    char *odd[] = {"bt_hex", "ABC"};
    unsigned char frame[64]; size_t frame_len; unsigned errors;
    assert(sizeof(s_uart_flags) == sizeof(rt_bool_t));
    assert(bt_test(2, on) == -RT_ERROR);
    s_uart_flags = BLE_UART_READY;
    assert(bt_send(2, text) == -RT_EBUSY && tx_len == 0);
    s_rx_length = 1;
    assert(bt_test(2, on) == 0 && s_rx_length == 0);
    assert(s_uart_flags == (BLE_UART_READY | BLE_UART_RAW_TEST));
    clear_io(); assert(bt_send(2, text) == 0);
    assert(tx_len == 6 && !memcmp(tx, "PING\r\n", 6));
    clear_io(); assert(bt_send(3, cr) == 0 && tx_len == 5 && tx[4] == '\r');
    clear_io(); assert(bt_send(3, none) == 0 && tx_len == 4);
    clear_io(); assert(bt_hex(2, hex) == 0);
    assert(tx_len == 5 && !memcmp(tx, "\x00\x55\xAA\x0D\x0A", 5));
    clear_io(); assert(bt_hex(2, bad) == -RT_EINVAL && tx_len == 0);
    assert(bt_hex(2, odd) == -RT_EINVAL && tx_len == 0);
    power_on = 0; assert(bt_send(2, text) == -RT_ERROR && tx_len == 0); power_on = 1;
    stall_write = 1; assert(bt_send(2, text) == -RT_ERROR && now == 5000 && tx_len == 0);
    stall_write = 0; clear_io(); errors = s_protocol_errors;
    ble_rx_process_chunk((const rt_uint8_t *)"\x00\x55\xAA\x0D\x0A", 5);
    assert(strstr(logs, "HEX: 00 55 AA 0D 0A") && strstr(logs, "ASCII:.U..."));
    assert(s_protocol_errors == errors && s_rx_length == 0 && dispatches == 0);
    clear_io(); assert(ble_ctrl2_send(2, 1, "ABC", 3) == 0 && tx_len == 0);
    s_ntn_pending = 1;
    assert(bt_test(2, off) == -RT_EBUSY && (s_uart_flags & BLE_UART_RAW_TEST));
    s_ntn_pending = 0; s_ntn_request_active = 1;
    assert(bt_test(2, off) == -RT_EBUSY);
    s_ntn_request_active = 0; assert(bt_test(2, off) == 0);
    assert(s_uart_flags == BLE_UART_READY);
    clear_io(); assert(ble_ctrl2_send(2, 7, "ABCD", 4) == 14);
    memcpy(frame, tx, tx_len); frame_len = tx_len; clear_io();
    ble_rx_process_chunk(frame, 1);
    ble_rx_process_chunk(frame + 1, 4);
    assert(dispatches == 0);
    ble_rx_process_chunk(frame + 5, frame_len - 5);
    assert(dispatches == 1 && s_rx_frames == 1 && s_rx_length == 0);
    assert(s_uart_flags == BLE_UART_READY); /* Reentrant switch was rejected. */
    frame[frame_len - 1] ^= 1;
    ble_rx_process_chunk(frame, frame_len);
    assert(s_crc_errors == 1 && dispatches == 1);
    s_rx_length = 1; s_rx_partial_tick = now; now += 5000; errors = s_protocol_errors;
    ble_rx_process_chunk(NULL, 0);
    assert(s_rx_length == 0 && s_protocol_errors == errors + 1);
    /* Echo firmware must not cause an ERROR -> echo -> ERROR feedback loop. */
    { unsigned type;
      memset(frame, 0, sizeof(frame));
      for (type = 2; type <= 8; type++) {
          if (type == 3) continue; /* Valid inbound NTN request, not an echo. */
          frame[3] = (unsigned char)type;
          ctrl2_handle_frame_production(frame, 0);
      }
      assert(outbound_errors == 0);
      frame[3] = 0xFF; ctrl2_handle_frame_production(frame, 0);
      assert(outbound_errors == 1);
      frame[3] = CTRL2_TYPE_LOCAL_FINAL; ctrl2_handle_frame_production(frame, 0);
      assert(outbound_errors == 1);
    }
    assert(!locked && !irq);
    puts("PASS: BLE raw text/HEX, partial TX, RX display, mode isolation, CTRL2 fragments/CRC/timeout");
    return 0;
}
'''


def main():
    compiler = os.environ.get("HOST_CC") or shutil.which("gcc")
    if not compiler:
        raise SystemExit("Set HOST_CC to the host GCC executable")
    source = (BSP / "applications/ble_uart.c").read_text(encoding="utf-8")
    header = (BSP / "applications/ble_uart.h").read_text(encoding="utf-8")
    constants = "\n".join(re.findall(r"^#define\s+(?:CTRL2_\w+|BLE_UART_(?:READY|RAW_TEST|RX_BUSY))\s+[^\n]+", header + "\n" + source, re.M))
    handler = function(source, "ctrl2_handle_frame").replace(
        "ctrl2_handle_frame(", "ctrl2_handle_frame_production(", 1)
    code = constants + "\n" + STUBS + handler
    code += "\n".join(function(source, name) for name in FUNCTIONS) + CASES
    with tempfile.TemporaryDirectory(prefix="ble_uart_") as directory:
        path = Path(directory) / "test.c"
        binary = Path(directory) / ("test.exe" if os.name == "nt" else "test")
        path.write_text(code, encoding="utf-8")
        subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-I", str(BSP / "applications"), str(path), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()

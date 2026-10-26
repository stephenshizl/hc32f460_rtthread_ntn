/* SPDX-License-Identifier: Apache-2.0 */

#include "ble_uart.h"
#include "ntn_link.h"
#include "bt_identity.h"
#include "hw_power.h"
#include "mcu_local_cmd.h"
#include "ntn_ms150.h"
#include "param_manager.h"
#include "pc_rs232.h"
#include "task_wdt.h"
#include <rthw.h>
#include <string.h>

#define DBG_TAG "ble.ctrl2"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define BLE_THREAD_STACK_SZ       5120U
#define BLE_THREAD_PRIO           10U
#define CTRL2_FRAME_OVERHEAD      10U
#define CTRL2_MAX_FRAME_SIZE      (CTRL2_MAX_PAYLOAD + CTRL2_FRAME_OVERHEAD)
#define BLE_UART_READY            0x01U
#define BLE_UART_RAW_TEST         0x02U
#define BLE_UART_RX_BUSY          0x04U

static ntnl_decoder s_link_decoder;
static rt_tick_t s_wire_last;
static rt_device_t s_ble_dev;
static struct rt_semaphore s_rx_sem;
static struct rt_mutex s_tx_lock;
static rt_thread_t s_rx_thread;
static rt_thread_t s_ntn_thread;
static struct rt_semaphore s_ntn_sem;
static volatile rt_bool_t s_ntn_pending;
static char s_ntn_cmd[CTRL2_MAX_PAYLOAD + 1U];
static rt_tick_t s_rx_partial_tick;
/* Reuse the former initialization word; diagnostics allocate no new RAM. */
static volatile rt_uint32_t s_uart_flags;
static rt_uint8_t s_rx_stream[CTRL2_RX_BUFFER_SIZE];
static rt_size_t s_rx_length;
static rt_uint8_t s_tx_frame[CTRL2_MAX_FRAME_SIZE];
static char s_local_response[CTRL2_MAX_PAYLOAD + 1U];
static char s_command[CTRL2_MAX_PAYLOAD + 1U];
static char s_event_json[1536];
static volatile rt_bool_t s_ntn_request_active;
static volatile rt_uint16_t s_ntn_request_id;
static volatile rt_uint32_t s_rx_frames;
static volatile rt_uint32_t s_crc_errors;
static volatile rt_uint32_t s_protocol_errors;

static rt_uint16_t ctrl2_crc16(const rt_uint8_t *data, rt_size_t len)
{
    rt_uint16_t crc = 0xFFFFU;
    rt_size_t i;
    for (i = 0U; i < len; i++)
    {
        rt_uint8_t bit;
        crc ^= (rt_uint16_t)data[i] << 8;
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 0x8000U) ? (rt_uint16_t)((crc << 1) ^ 0x1021U) :
                                    (rt_uint16_t)(crc << 1);
        }
    }
    return crc;
}

static rt_err_t ble_rx_ind(rt_device_t dev, rt_size_t size)
{
    RT_UNUSED(dev);
    RT_UNUSED(size);
    rt_sem_release(&s_rx_sem);
    return RT_EOK;
}

/* Serial drivers are allowed to accept a partial write; never emit a
 * truncated CTRL2 frame and pretend it was fully sent. */
static rt_size_t ble_write_all_locked(const rt_uint8_t *buf, rt_size_t len)
{
    rt_size_t sent = 0U;
    rt_tick_t start = rt_tick_get();
    while (sent < len)
    {
        rt_size_t n = rt_device_write(s_ble_dev, 0, buf + sent, len - sent);
        sent += n;
        if (n == 0U)
        {
            if ((rt_tick_get() - start) >= rt_tick_from_millisecond(5000U)) break;
            rt_thread_mdelay(1U);
        }
    }
    return sent;
}

/* Caller owns the existing TX mutex across all chunks of one CTRL2 frame. */
static rt_size_t ble_data_write_locked(const rt_uint8_t *buf,rt_size_t len)
{
    rt_uint8_t wire[NTNL_WIRE_MAX];rt_size_t sent=0;
    while (sent<len) {
        rt_uint16_t n=(len-sent>NTNL_MAX_PAYLOAD)?NTNL_MAX_PAYLOAD:(rt_uint16_t)(len-sent);
        unsigned total=ntnl_encode(NTNL_DATA,0,buf+sent,n,wire);
        if (ble_write_all_locked(wire,total)!=total) break;
        sent+=n;
    }
    return sent;
}
rt_size_t ble_management_send(rt_uint8_t kind,rt_uint16_t id,const void *data,rt_uint16_t len)
{
    rt_uint8_t wire[NTNL_WIRE_MAX];unsigned total;rt_size_t written=0;
    if (!(s_uart_flags & BLE_UART_READY) || len>NTNL_MAX_PAYLOAD) return 0;
    total=ntnl_encode(kind,id,data,len,wire);
    if (rt_mutex_take(&s_tx_lock,RT_WAITING_FOREVER)!=RT_EOK) return 0;
    written=ble_write_all_locked(wire,total);rt_mutex_release(&s_tx_lock);
    return written==total?len+1U:0U;
}

rt_size_t ble_uart_send(const void *buf, rt_size_t len)
{
    rt_size_t sent = 0U;
    if (!(s_uart_flags & BLE_UART_READY) || s_ble_dev == RT_NULL || buf == RT_NULL || len == 0U)
        return 0U;
    if (rt_mutex_take(&s_tx_lock, RT_WAITING_FOREVER) == RT_EOK)
    {
        sent = ble_data_write_locked((const rt_uint8_t *)buf, len);
        rt_mutex_release(&s_tx_lock);
    }
    return sent;
}

rt_size_t ble_ctrl2_send(rt_uint8_t type, rt_uint16_t request_id,
                         const void *payload, rt_uint16_t payload_len)
{
    rt_uint16_t crc;
    rt_size_t total;
    rt_size_t sent;
    if (!(s_uart_flags & BLE_UART_READY) || payload_len > CTRL2_MAX_PAYLOAD ||
        (payload_len > 0U && payload == RT_NULL)) return 0U;
    total = CTRL2_FRAME_OVERHEAD + payload_len;
    if (rt_mutex_take(&s_tx_lock, RT_WAITING_FOREVER) != RT_EOK) return 0U;
    /* Do not interleave asynchronous CTRL2 traffic with raw UART diagnostics. */
    if (s_uart_flags & BLE_UART_RAW_TEST)
    {
        rt_mutex_release(&s_tx_lock);
        return 0U;
    }
    s_tx_frame[0] = CTRL2_SYNC_1;
    s_tx_frame[1] = CTRL2_SYNC_2;
    s_tx_frame[2] = CTRL2_VERSION;
    s_tx_frame[3] = type;
    s_tx_frame[4] = (rt_uint8_t)(request_id >> 8);
    s_tx_frame[5] = (rt_uint8_t)request_id;
    s_tx_frame[6] = (rt_uint8_t)(payload_len >> 8);
    s_tx_frame[7] = (rt_uint8_t)payload_len;
    if (payload_len > 0U) rt_memcpy(&s_tx_frame[8], payload, payload_len);
    crc = ctrl2_crc16(&s_tx_frame[2], 6U + payload_len);
    s_tx_frame[8U + payload_len] = (rt_uint8_t)(crc >> 8);
    s_tx_frame[9U + payload_len] = (rt_uint8_t)crc;
    sent = ble_data_write_locked(s_tx_frame, total);
    rt_mutex_release(&s_tx_lock);
    return sent;
}

static rt_size_t ctrl2_trim_line(const rt_uint8_t *payload, rt_uint16_t length,
                                 char *out, rt_size_t out_size)
{
    rt_size_t len = length;
    if (len == 0U || len >= out_size) return 0U;
    while (len > 0U && (payload[len - 1U] == '\r' || payload[len - 1U] == '\n')) len--;
    if (len == 0U || len >= out_size) return 0U;
    rt_memcpy(out, payload, len);
    out[len] = '\0';
    return len;
}

static rt_bool_t ctrl2_is_printable(const rt_uint8_t *payload, rt_size_t len)
{
    rt_size_t i;
    for (i = 0U; i < len; i++)
        if (payload[i] < 0x20U || payload[i] > 0x7EU) return RT_FALSE;
    return RT_TRUE;
}

static void ctrl2_send_text(rt_uint8_t type, rt_uint16_t request_id,
                            const char *text)
{
    ble_ctrl2_send(type, request_id, text, (rt_uint16_t)rt_strlen(text));
}

static void ctrl2_apply_baud(rt_uint16_t request_id,
                             const mcu_cmd_action_t *action)
{
    rt_tick_t deadline;
    rt_err_t result = -RT_EBUSY;
    ctrl2_send_text(CTRL2_TYPE_LOCAL_PENDING, request_id, s_local_response);
    deadline = rt_tick_get() + rt_tick_from_millisecond(2000U);
    while ((rt_int32_t)(rt_tick_get() - deadline) < 0)
    {
        if (pc_rs232_is_idle(100U))
        {
            result = pc_rs232_set_baudrate(action->value);
            break;
        }
        rt_thread_mdelay(20U);
    }
    if (result == RT_EOK)
    {
        rt_snprintf(s_local_response, sizeof(s_local_response),
                    "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"baud\":%u}\r\nOK\r\n",
                    action->value);
    }
    else
    {
        (void)param_set_rs232_baud(action->previous_value, RT_TRUE);
        rt_snprintf(s_local_response, sizeof(s_local_response),
                    "\r\n+MCUERROR:%u\r\nERROR\r\n",
                    (result == -RT_EBUSY) ? 6U : 3U);
    }
    ctrl2_send_text(CTRL2_TYPE_LOCAL_FINAL, request_id, s_local_response);
}

static void ctrl2_handle_local(rt_uint16_t request_id,
                               const rt_uint8_t *payload, rt_uint16_t length)
{
    mcu_cmd_action_t action;
    rt_size_t line_len;
    line_len = ctrl2_trim_line(payload, length, s_command, sizeof(s_command));
    /* Local requests MUST be one printable ASCII command terminated by CRLF. */
    if (length < 2U || payload[length - 2U] != '\r' ||
        payload[length - 1U] != '\n' ||
        request_id == 0U || line_len == 0U ||
        memchr(payload, 0, length) != RT_NULL ||
        memchr(payload, '\r', length - 2U) != RT_NULL ||
        memchr(payload, '\n', length - 2U) != RT_NULL ||
        !ctrl2_is_printable(payload, length - 2U) ||
        !mcu_local_cmd_execute((const rt_uint8_t *)s_command, line_len,
                               s_local_response, sizeof(s_local_response), &action))
    {
        ctrl2_send_text(CTRL2_TYPE_LOCAL_FINAL, request_id,
                        "\r\n+MCUERROR:1\r\nERROR\r\n");
        return;
    }
    if (action.type == MCU_CMD_ACTION_APPLY_RS232_BAUD)
        ctrl2_apply_baud(request_id, &action);
    else if (action.type == MCU_CMD_ACTION_ENTER_PC_CONFIG ||
             action.type == MCU_CMD_ACTION_EXIT_PC_CONFIG)
        ctrl2_send_text(CTRL2_TYPE_LOCAL_FINAL, request_id,
                        "\r\n+MCUERROR:5\r\nERROR\r\n");
    else
    {
        ctrl2_send_text(CTRL2_TYPE_LOCAL_FINAL, request_id, s_local_response);
        mcu_local_cmd_apply_reboot(&action);
    }
}

static void ctrl2_ntn_rx(const rt_uint8_t *data, rt_size_t len,
                         rt_bool_t command_associated)
{
    rt_uint8_t type = CTRL2_TYPE_NTN_ASYNC;
    rt_uint16_t request_id = 0U;
    if (command_associated && s_ntn_request_active)
    {
        type = CTRL2_TYPE_NTN_FRAGMENT;
        request_id = s_ntn_request_id;
    }
    while (len > 0U)
    {
        rt_uint16_t chunk = (len > CTRL2_MAX_PAYLOAD) ?
                            CTRL2_MAX_PAYLOAD : (rt_uint16_t)len;
        ble_ctrl2_send(type, request_id, data, chunk);
        data += chunk;
        len -= chunk;
    }
}

static void ctrl2_downlink_event(const ntn_downlink_t *downlink)
{
    static const char hex[] = "0123456789ABCDEF";
    rt_size_t pos;
    rt_uint16_t i;
    if (downlink == RT_NULL || !downlink->valid) return;
    pos = (rt_size_t)rt_snprintf(s_event_json, sizeof(s_event_json),
                                 "{\"event\":\"NTN_DOWNLINK\",\"socket_id\":%u,"
                                 "\"reported_len\":%u,\"decoded_len\":%u,\"hex\":\"",
                                 downlink->socket_id, downlink->data_len,
                                 downlink->data_len);
    for (i = 0U; i < downlink->data_len && pos + 3U < sizeof(s_event_json); i++)
    {
        s_event_json[pos++] = hex[downlink->data[i] >> 4];
        s_event_json[pos++] = hex[downlink->data[i] & 0x0FU];
    }
    if (pos + 3U < sizeof(s_event_json))
    {
        s_event_json[pos++] = '"';
        s_event_json[pos++] = '}';
        s_event_json[pos] = '\0';
        ble_ctrl2_send(CTRL2_TYPE_EVENT, 0U, s_event_json, (rt_uint16_t)pos);
    }
}

/* NTN commands run on a separate worker: the CTRL2 RX thread must keep draining
 * UART4 and feeding the watchdog during a 60-second satellite transaction. */
static void ctrl2_ntn_worker(void *parameter)
{
    RT_UNUSED(parameter);
    for (;;)
    {
        rt_err_t result;
        char final[64];
        if (rt_sem_take(&s_ntn_sem,rt_tick_from_millisecond(2000U))!=RT_EOK) {
            rt_base_t level=rt_hw_interrupt_disable();
            if (s_ntn_pending) {rt_hw_interrupt_enable(level);continue;}
            s_ntn_pending=RT_TRUE;rt_hw_interrupt_enable(level);
            bt_identity_poll(s_ntn_cmd,sizeof(s_ntn_cmd));
            s_ntn_pending=RT_FALSE;continue;
        }
        if (ntn_debug_lock_acquire() != RT_EOK)
        {
            ctrl2_send_text(CTRL2_TYPE_NTN_FINAL, s_ntn_request_id,
                            "{\"ok\":false,\"error\":6}");
            s_ntn_pending = RT_FALSE;
            continue;
        }
        s_ntn_request_active = RT_TRUE;
        /* All response bytes are already streamed as NTN_FRAGMENT frames.
         * Do not impose a second 96-byte capture limit on the final result. */
        result = ntn_debug_exec_cmd(s_ntn_cmd, RT_NULL, 0U, 60000U);
        s_ntn_request_active = RT_FALSE;
        /* A completed Bluetooth debug transaction must immediately return
         * UART3 ownership to the automatic NTN service.  Waiting for the
         * 60-second lease deadline would stall normal uplink after every
         * successful short AT command. */
        ntn_debug_lock_release();
        rt_snprintf(final, sizeof(final), "{\"ok\":%s,\"error\":%u}",
                    result == RT_EOK ? "true" : "false",
                    result == RT_EOK ? 0U :
                    (result == -RT_ETIMEOUT ? 12U :
                     (result == -RT_EBUSY ? 6U : 4U)));
        ctrl2_send_text(CTRL2_TYPE_NTN_FINAL, s_ntn_request_id, final);
        s_ntn_pending = RT_FALSE;
    }
}

static void ctrl2_handle_ntn(rt_uint16_t request_id,
                             const rt_uint8_t *payload, rt_uint16_t length)
{
    rt_size_t command_len = ctrl2_trim_line(payload, length, s_command,
                                             sizeof(s_command));
    if (request_id == 0U || command_len < 2U ||
        length == 0U || (payload[length - 1U] != '\r' &&
                         payload[length - 1U] != '\n') ||
        memchr(payload, 0, length) != RT_NULL ||
        !ctrl2_is_printable(payload, command_len) ||
        s_command[0] != 'A' || s_command[1] != 'T')
    {
        ctrl2_send_text(CTRL2_TYPE_NTN_FINAL, request_id,
                        "{\"ok\":false,\"error\":1}");
        return;
    }
    if (s_ntn_pending)
    {
        ctrl2_send_text(CTRL2_TYPE_NTN_FINAL, request_id,
                        "{\"ok\":false,\"error\":6}");
        return;
    }
    rt_memcpy(s_ntn_cmd, s_command, command_len + 1U);
    s_ntn_request_id = request_id;
    s_ntn_pending = RT_TRUE;
    rt_sem_release(&s_ntn_sem);
}

static void ctrl2_handle_frame(const rt_uint8_t *frame, rt_uint16_t payload_len)
{
    rt_uint8_t type = frame[3];
    rt_uint16_t request_id = ((rt_uint16_t)frame[4] << 8) | frame[5];
    const rt_uint8_t *payload = &frame[8];
    if (type == CTRL2_TYPE_LOCAL_REQUEST)
        ctrl2_handle_local(request_id, payload, payload_len);
    else if (type == CTRL2_TYPE_NTN_REQUEST)
        ctrl2_handle_ntn(request_id, payload, payload_len);
    else
    {
        s_protocol_errors++;
        /* These are MCU -> phone responses/events, never inbound requests.
         * Ignore a local echo instead of replying ERROR to ERROR forever. */
        if (type == CTRL2_TYPE_LOCAL_FINAL || type == CTRL2_TYPE_NTN_FRAGMENT ||
            type == CTRL2_TYPE_NTN_ASYNC || type == CTRL2_TYPE_EVENT ||
            type == CTRL2_TYPE_NTN_FINAL || type == CTRL2_TYPE_LOCAL_PENDING)
            return;
        ctrl2_send_text(CTRL2_TYPE_LOCAL_FINAL, request_id,
                        "\r\n+MCUERROR:18\r\nERROR\r\n");
    }
}

static void ctrl2_drop_prefix(rt_size_t count)
{
    if (count >= s_rx_length)
    {
        s_rx_length = 0U;
        return;
    }
    rt_memmove(s_rx_stream, &s_rx_stream[count], s_rx_length - count);
    s_rx_length -= count;
}

static void ctrl2_parse_stream(void)
{
    while (s_rx_length >= 2U)
    {
        rt_size_t sync = 0U;
        rt_uint16_t payload_len;
        rt_size_t frame_len;
        rt_uint16_t expected_crc;
        rt_uint16_t actual_crc;
        while (sync + 1U < s_rx_length &&
               !(s_rx_stream[sync] == CTRL2_SYNC_1 &&
                 s_rx_stream[sync + 1U] == CTRL2_SYNC_2)) sync++;
        if (sync > 0U) ctrl2_drop_prefix(sync);
        if (s_rx_length < CTRL2_FRAME_OVERHEAD) return;
        if (s_rx_stream[2] != CTRL2_VERSION)
        {
            ctrl2_drop_prefix(1U);
            s_protocol_errors++;
            continue;
        }
        payload_len = ((rt_uint16_t)s_rx_stream[6] << 8) | s_rx_stream[7];
        if (payload_len > CTRL2_MAX_PAYLOAD)
        {
            ctrl2_drop_prefix(1U);
            s_protocol_errors++;
            continue;
        }
        frame_len = CTRL2_FRAME_OVERHEAD + payload_len;
        if (s_rx_length < frame_len) return;
        expected_crc = ((rt_uint16_t)s_rx_stream[8U + payload_len] << 8) |
                       s_rx_stream[9U + payload_len];
        actual_crc = ctrl2_crc16(&s_rx_stream[2], 6U + payload_len);
        if (expected_crc != actual_crc)
        {
            s_crc_errors++;
            ctrl2_drop_prefix(1U);
            continue;
        }
        s_rx_frames++;
        ctrl2_handle_frame(s_rx_stream, payload_len);
        ctrl2_drop_prefix(frame_len);
    }
}

static void ble_test_dump_rx(const rt_uint8_t *data, rt_size_t len)
{
    rt_size_t offset;
    for (offset = 0U; offset < len; offset += 16U)
    {
        rt_size_t i;
        rt_size_t count = (len - offset > 16U) ? 16U : len - offset;
        rt_kprintf("[BT RX %u] HEX:", (unsigned)count);
        for (i = 0U; i < count; i++) rt_kprintf(" %02X", data[offset + i]);
        rt_kprintf("  ASCII:");
        for (i = 0U; i < count; i++)
        {
            rt_uint8_t ch = data[offset + i];
            rt_kprintf("%c", ch >= 0x20U && ch <= 0x7EU ? ch : '.');
        }
        rt_kprintf("\n");
    }
}

static void ble_rx_process_chunk(const rt_uint8_t *chunk, rt_size_t count)
{
    rt_uint32_t flags;
    rt_base_t level = rt_hw_interrupt_disable();
    flags = s_uart_flags;
    s_uart_flags |= BLE_UART_RX_BUSY;
    rt_hw_interrupt_enable(level);

    if (flags & BLE_UART_RAW_TEST)
    {
        /* Use the original RX thread/chunk; do not feed test bytes to CTRL2. */
        ble_test_dump_rx(chunk, count);
    }
    else
    {
        if (s_rx_length != 0U &&
            (rt_tick_get() - s_rx_partial_tick) >= rt_tick_from_millisecond(5000U))
        {
            s_rx_length = 0U;
            s_protocol_errors++;
        }
        if (count > sizeof(s_rx_stream) - s_rx_length)
        {
            s_rx_length = 0U;
            s_protocol_errors++;
        }
        if (count > 0U && count <= sizeof(s_rx_stream) - s_rx_length)
        {
            rt_memcpy(&s_rx_stream[s_rx_length], chunk, count);
            s_rx_length += count;
            s_rx_partial_tick = rt_tick_get();
            ctrl2_parse_stream();
        }
    }
    level = rt_hw_interrupt_disable();
    s_uart_flags &= ~BLE_UART_RX_BUSY;
    rt_hw_interrupt_enable(level);
}

static void ble_wire_received(unsigned char kind,unsigned short id,
                              const unsigned char *payload,unsigned short len)
{
    if (kind==NTNL_DATA) ble_rx_process_chunk(payload,len);
    else if (kind==NTNL_REPLY) bt_identity_link_reply(id,payload,len);
}
static void ble_wire_input(const rt_uint8_t *chunk,rt_size_t count)
{
    rt_tick_t now=rt_tick_get();
    if ((now-s_wire_last)>rt_tick_from_millisecond(1000)) ntnl_expire(&s_link_decoder);
    if (count) {ntnl_feed(&s_link_decoder,chunk,(unsigned)count,ble_wire_received);s_wire_last=now;}
}

static void ble_rx_thread_entry(void *parameter)
{
    rt_uint8_t chunk[128];
    RT_UNUSED(parameter);
    while (1)
    {
        rt_size_t count;
        (void)rt_sem_take(&s_rx_sem, rt_tick_from_millisecond(5000U));
        task_wdt_heartbeat_by_name("ble");
        ble_rx_process_chunk(RT_NULL, 0U);
        ble_wire_input(RT_NULL,0U);
        while ((count = rt_device_read(s_ble_dev, 0, chunk, sizeof(chunk))) > 0U)
        {
            ble_wire_input(chunk, count);
        }
    }
}

int ble_uart_init(void)
{
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;
    if (s_uart_flags & BLE_UART_READY) return RT_EOK;
    hw_power_bt_set(RT_TRUE);
    s_ble_dev = rt_device_find(BLE_UART_NAME);
    if (s_ble_dev == RT_NULL) return -RT_ERROR;
    cfg.baud_rate = BLE_DEFAULT_BAUDRATE;
    cfg.data_bits = DATA_BITS_8;
    cfg.stop_bits = STOP_BITS_1;
    cfg.parity = PARITY_NONE;
    if (rt_device_control(s_ble_dev, RT_DEVICE_CTRL_CONFIG, &cfg) != RT_EOK ||
        rt_device_open(s_ble_dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
        return -RT_ERROR;
    rt_sem_init(&s_rx_sem, "sctrl2", 0, RT_IPC_FLAG_PRIO);
    rt_sem_init(&s_ntn_sem, "sbntn", 0, RT_IPC_FLAG_PRIO);
    rt_mutex_init(&s_tx_lock, "mctrl2", RT_IPC_FLAG_PRIO);
    rt_device_set_rx_indicate(s_ble_dev, ble_rx_ind);
    s_uart_flags = BLE_UART_READY;
    ntn_set_raw_rx_handler(ctrl2_ntn_rx);
    ntn_set_downlink_handler(ctrl2_downlink_event);
    task_wdt_register("ble", 10U);
    s_rx_thread = rt_thread_create("ble_ctrl2", ble_rx_thread_entry, RT_NULL,
                                   BLE_THREAD_STACK_SZ, BLE_THREAD_PRIO, 10U);
    if (s_rx_thread == RT_NULL)
    {
        s_uart_flags = 0U;
        return -RT_ERROR;
    }
    rt_thread_startup(s_rx_thread);
    s_ntn_thread = rt_thread_create("btntn", ctrl2_ntn_worker, RT_NULL,
                                    3072U, BLE_THREAD_PRIO + 1U, 10U);
    if (s_ntn_thread == RT_NULL) return -RT_ERROR;
    rt_thread_startup(s_ntn_thread);
    LOG_I("CTRL2 ready: v2, request-id16, payload<=2048, CRC16/CCITT-FALSE");
    return RT_EOK;
}
INIT_ENV_EXPORT(ble_uart_init);

#ifdef RT_USING_FINSH
#include <finsh.h>
static int bt_test(int argc, char **argv)
{
    rt_base_t level;
    rt_bool_t enabled;
    if (argc != 2 || (rt_strcmp(argv[1], "on") != 0 && rt_strcmp(argv[1], "off") != 0))
    {
        rt_kprintf("Usage: bt_test <on|off>\n");
        return -RT_EINVAL;
    }
    if (!(s_uart_flags & BLE_UART_READY)) return -RT_ERROR;
    enabled = rt_strcmp(argv[1], "on") == 0;
    if (rt_mutex_take(&s_tx_lock, RT_WAITING_FOREVER) != RT_EOK) return -RT_EBUSY;
    level = rt_hw_interrupt_disable();
    if ((s_uart_flags & BLE_UART_RX_BUSY) || s_ntn_pending || s_ntn_request_active)
    {
        rt_hw_interrupt_enable(level);
        rt_mutex_release(&s_tx_lock);
        rt_kprintf("BT busy; finish the active CTRL2 transaction and retry.\n");
        return -RT_EBUSY;
    }
    if (enabled) s_uart_flags |= BLE_UART_RAW_TEST;
    else s_uart_flags &= ~BLE_UART_RAW_TEST;
    s_rx_length = 0U; /* Mode changes intentionally discard a partial frame. */
    rt_hw_interrupt_enable(level);
    rt_mutex_release(&s_tx_lock);
    rt_kprintf("BT uart4 PB7(TX)/PB6(RX), 115200/8N1: %s, power=%s\n",
               enabled ? "RAW TEST (CTRL2 paused)" : "CTRL2 restored",
               hw_power_bt_get() ? "ON" : "OFF");
    return RT_EOK;
}
MSH_CMD_EXPORT(bt_test, raw Bluetooth UART diagnostics <on|off> no new RAM);

static int ble_test_tx_lock(void)
{
    if (!(s_uart_flags & BLE_UART_READY) || !hw_power_bt_get())
    {
        rt_kprintf("BT UART not ready or power OFF; check power_bt on.\n");
        return -RT_ERROR;
    }
    if (rt_mutex_take(&s_tx_lock, RT_WAITING_FOREVER) != RT_EOK) return -RT_EBUSY;
    if (!(s_uart_flags & BLE_UART_RAW_TEST))
    {
        rt_mutex_release(&s_tx_lock);
        rt_kprintf("Run bt_test on first.\n");
        return -RT_EBUSY;
    }
    return RT_EOK;
}

static int bt_send(int argc, char **argv)
{
    const char *ending = "\r\n";
    rt_size_t ending_len = 2U;
    rt_size_t len;
    rt_size_t sent;
    int result;
    if (argc < 2 || argc > 3) goto usage;
    if (argc == 3)
    {
        if (rt_strcmp(argv[2], "none") == 0) ending_len = 0U;
        else if (rt_strcmp(argv[2], "cr") == 0) ending_len = 1U;
        else if (rt_strcmp(argv[2], "crlf") != 0) goto usage;
    }
    result = ble_test_tx_lock();
    if (result != RT_EOK) return result;
    len = rt_strlen(argv[1]);
    sent = ble_data_write_locked((const rt_uint8_t *)argv[1], len);
    if (sent == len)
        sent += ble_data_write_locked((const rt_uint8_t *)ending, ending_len);
    rt_mutex_release(&s_tx_lock);
    rt_kprintf("[BT TX] %u/%u bytes; this is TX only, check BT RX for reply.\n",
               (unsigned)sent, (unsigned)(len + ending_len));
    return sent == len + ending_len ? RT_EOK : -RT_ERROR;
usage:
    rt_kprintf("Usage: bt_send <text> [crlf|cr|none] (default crlf)\n");
    return -RT_EINVAL;
}
MSH_CMD_EXPORT(bt_send, send raw text to Bluetooth UART default CRLF);

static int ble_test_hex_nibble(char ch)
{
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    return -1;
}

static int bt_hex(int argc, char **argv)
{
    rt_size_t len;
    rt_size_t i;
    int result;
    if (argc != 2) goto usage;
    len = rt_strlen(argv[1]);
    if (len == 0U || (len & 1U) != 0U) goto usage;
    /* Validate everything before sending; invalid input must emit no bytes. */
    for (i = 0U; i < len; i++) if (ble_test_hex_nibble(argv[1][i]) < 0) goto usage;
    result = ble_test_tx_lock();
    if (result != RT_EOK) return result;
    for (i = 0U; i < len; i += 2U)
    {
        rt_uint8_t ch = (rt_uint8_t)((ble_test_hex_nibble(argv[1][i]) << 4) |
                                    ble_test_hex_nibble(argv[1][i + 1U]));
        if (ble_data_write_locked(&ch, 1U) != 1U) break;
    }
    rt_mutex_release(&s_tx_lock);
    rt_kprintf("[BT TX HEX] %u/%u bytes; check BT RX for reply.\n",
               (unsigned)(i / 2U), (unsigned)(len / 2U));
    return i == len ? RT_EOK : -RT_ERROR;
usage:
    rt_kprintf("Usage: bt_hex <even-length HEX, e.g. 0055AA0D0A>\n");
    return -RT_EINVAL;
}
MSH_CMD_EXPORT(bt_hex, send raw HEX bytes to Bluetooth UART without extra CRLF);

static void ctrl2_status(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    rt_kprintf("ctrl2 frames=%u crc_err=%u proto_err=%u buffered=%u ntn_lock=%u\n",
               s_rx_frames, s_crc_errors, s_protocol_errors, s_rx_length,
               ntn_debug_lock_is_held());
    rt_kprintf("uart4 115200/8N1, raw_test=%u, power=%s\n",
               (s_uart_flags & BLE_UART_RAW_TEST) ? 1U : 0U,
               hw_power_bt_get() ? "ON" : "OFF");
}
MSH_CMD_EXPORT(ctrl2_status, show MCU Bluetooth CTRL2 status);
#endif

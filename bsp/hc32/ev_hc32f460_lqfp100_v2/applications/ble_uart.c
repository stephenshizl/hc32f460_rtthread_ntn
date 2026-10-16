/* SPDX-License-Identifier: Apache-2.0 */

#include "ble_uart.h"
#include "hw_power.h"
#include "mcu_local_cmd.h"
#include "ntn_ms150.h"
#include "param_manager.h"
#include "pc_rs232.h"
#include "task_wdt.h"
#include <string.h>

#define DBG_TAG "ble.ctrl2"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define BLE_THREAD_STACK_SZ       5120U
#define BLE_THREAD_PRIO           10U
#define CTRL2_FRAME_OVERHEAD      10U
#define CTRL2_MAX_FRAME_SIZE      (CTRL2_MAX_PAYLOAD + CTRL2_FRAME_OVERHEAD)

static rt_device_t s_ble_dev;
static struct rt_semaphore s_rx_sem;
static struct rt_mutex s_tx_lock;
static rt_thread_t s_rx_thread;
static rt_bool_t s_initialized;
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

rt_size_t ble_uart_send(const void *buf, rt_size_t len)
{
    rt_size_t sent = 0U;
    if (!s_initialized || s_ble_dev == RT_NULL || buf == RT_NULL || len == 0U)
        return 0U;
    if (rt_mutex_take(&s_tx_lock, RT_WAITING_FOREVER) == RT_EOK)
    {
        sent = rt_device_write(s_ble_dev, 0, buf, len);
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
    if (!s_initialized || payload_len > CTRL2_MAX_PAYLOAD ||
        (payload_len > 0U && payload == RT_NULL)) return 0U;
    total = CTRL2_FRAME_OVERHEAD + payload_len;
    if (rt_mutex_take(&s_tx_lock, RT_WAITING_FOREVER) != RT_EOK) return 0U;
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
    sent = rt_device_write(s_ble_dev, 0, s_tx_frame, total);
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
    if (request_id == 0U || line_len == 0U ||
        !mcu_local_cmd_execute((const rt_uint8_t *)s_command, line_len,
                               s_local_response, sizeof(s_local_response), &action))
    {
        ctrl2_send_text(CTRL2_TYPE_LOCAL_FINAL, request_id,
                        "\r\n+MCUERROR:1\r\nERROR\r\n");
        return;
    }
    if (action.type == MCU_CMD_ACTION_APPLY_RS232_BAUD)
        ctrl2_apply_baud(request_id, &action);
    else
        ctrl2_send_text(CTRL2_TYPE_LOCAL_FINAL, request_id, s_local_response);
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

static void ctrl2_handle_ntn(rt_uint16_t request_id,
                             const rt_uint8_t *payload, rt_uint16_t length)
{
    rt_size_t command_len;
    rt_err_t result;
    char final[64];
    char response[64];
    command_len = ctrl2_trim_line(payload, length, s_command, sizeof(s_command));
    if (request_id == 0U || command_len == 0U)
    {
        ctrl2_send_text(CTRL2_TYPE_NTN_FINAL, request_id,
                        "{\"ok\":false,\"error\":1}");
        return;
    }
    if (ntn_debug_lock_acquire() != RT_EOK)
    {
        ctrl2_send_text(CTRL2_TYPE_NTN_FINAL, request_id,
                        "{\"ok\":false,\"error\":6}");
        return;
    }
    s_ntn_request_id = request_id;
    s_ntn_request_active = RT_TRUE;
    result = ntn_exec_cmd(s_command, response, sizeof(response), 60000U);
    s_ntn_request_active = RT_FALSE;
    rt_snprintf(final, sizeof(final), "{\"ok\":%s,\"error\":%d}",
                result == RT_EOK ? "true" : "false", result);
    ctrl2_send_text(CTRL2_TYPE_NTN_FINAL, request_id, final);
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

static void ble_rx_thread_entry(void *parameter)
{
    rt_uint8_t chunk[128];
    RT_UNUSED(parameter);
    while (1)
    {
        rt_size_t count;
        (void)rt_sem_take(&s_rx_sem, rt_tick_from_millisecond(5000U));
        task_wdt_heartbeat_by_name("ble");
        while ((count = rt_device_read(s_ble_dev, 0, chunk, sizeof(chunk))) > 0U)
        {
            if (count > sizeof(s_rx_stream) - s_rx_length)
            {
                s_rx_length = 0U;
                s_protocol_errors++;
            }
            if (count <= sizeof(s_rx_stream) - s_rx_length)
            {
                rt_memcpy(&s_rx_stream[s_rx_length], chunk, count);
                s_rx_length += count;
                ctrl2_parse_stream();
            }
        }
    }
}

int ble_uart_init(void)
{
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;
    if (s_initialized) return RT_EOK;
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
    rt_mutex_init(&s_tx_lock, "mctrl2", RT_IPC_FLAG_PRIO);
    rt_device_set_rx_indicate(s_ble_dev, ble_rx_ind);
    s_initialized = RT_TRUE;
    ntn_set_raw_rx_handler(ctrl2_ntn_rx);
    ntn_set_downlink_handler(ctrl2_downlink_event);
    task_wdt_register("ble", 10U);
    s_rx_thread = rt_thread_create("ble_ctrl2", ble_rx_thread_entry, RT_NULL,
                                   BLE_THREAD_STACK_SZ, BLE_THREAD_PRIO, 10U);
    if (s_rx_thread == RT_NULL)
    {
        s_initialized = RT_FALSE;
        return -RT_ERROR;
    }
    rt_thread_startup(s_rx_thread);
    LOG_I("CTRL2 ready: v2, request-id16, payload<=2048, CRC16/CCITT-FALSE");
    return RT_EOK;
}
INIT_ENV_EXPORT(ble_uart_init);

#ifdef RT_USING_FINSH
#include <finsh.h>
static void ctrl2_status(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    rt_kprintf("ctrl2 frames=%u crc_err=%u proto_err=%u buffered=%u ntn_lock=%u\n",
               s_rx_frames, s_crc_errors, s_protocol_errors, s_rx_length,
               ntn_debug_lock_is_held());
}
MSH_CMD_EXPORT(ctrl2_status, show MCU Bluetooth CTRL2 status);
#endif

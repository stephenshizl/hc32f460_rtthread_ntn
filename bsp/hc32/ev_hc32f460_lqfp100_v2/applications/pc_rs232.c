/*
 * Copyright (c) 2026, youyoutech.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-13     Stephen      First version: PC RS232 Driver with idle frame detection
 */

#include "pc_rs232.h"
#include "param_manager.h"
#include <stdlib.h>
#include "task_wdt.h"

#define DBG_TAG "pc.rs232"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* This thread also executes local AT commands and synchronous NV saves.
 * ARMCC reports >2 KB for mode-0 saves and >5 KB through the mode-1
 * maintenance callback (including its response buffer). Reserve headroom
 * for indirect FAL/SFUD/SPI calls and the RTOS context frame.
 */
#define RS232_THREAD_STACK_SZ   8192
#define RS232_THREAD_PRIO       9

static rt_device_t            _serial_dev       = RT_NULL;
static struct rt_semaphore    _sem_rx;
static struct rt_mutex        _mutex_tx;
static rt_thread_t            _rx_thread        = RT_NULL;

static rt_uint8_t             _rx_frame_buf[RS232_RX_FRAME_MAX_SZ];
static pc_rs232_rx_handler_t  _rx_handler       = RT_NULL;
static rt_bool_t              _loopback_enabled = RT_FALSE;
static rt_bool_t              _inited           = RT_FALSE;

static rt_uint32_t            _current_baud     = RS232_DEFAULT_BAUDRATE;
static volatile rt_uint32_t   _stat_rx_frames   = 0;
static volatile rt_uint32_t   _stat_rx_bytes    = 0;
static volatile rt_uint32_t   _stat_tx_frames   = 0;
static volatile rt_uint32_t   _stat_tx_bytes    = 0;
static volatile rt_tick_t     _last_activity_tick;

static void rs232_dispatch_chunk(rt_size_t frame_len)
{
    if (frame_len == 0U)
    {
        return;
    }

    _stat_rx_frames++;
    _stat_rx_bytes += frame_len;
    _last_activity_tick = rt_tick_get();
    if (_loopback_enabled)
    {
        pc_rs232_send(_rx_frame_buf, frame_len);
    }
    if (_rx_handler != RT_NULL)
    {
        _rx_handler(_rx_frame_buf, frame_len);
    }
}

/**
 * @brief Serial RX interrupt indication callback
 */
static rt_err_t rs232_rx_ind(rt_device_t dev, rt_size_t size)
{
    RT_UNUSED(dev);
    RT_UNUSED(size);

    /* Release semaphore to wake up RX frame assembling thread */
    rt_sem_release(&_sem_rx);
    return RT_EOK;
}

/**
 * @brief Receiver thread with inter-character idle timeout framing
 */
static void rs232_rx_thread_entry(void *parameter)
{
    RT_UNUSED(parameter);

    while (1)
    {
        /* Block until the first byte of a new frame arrives or 5s heartbeat timeout */
        if (rt_sem_take(&_sem_rx, rt_tick_from_millisecond(5000)) != RT_EOK)
        {
            task_wdt_heartbeat_by_name("rs232");
            continue;
        }
        task_wdt_heartbeat_by_name("rs232");

        rt_size_t frame_len = 0;

        while (1)
        {
            rt_uint8_t ch;
            /* Read all currently available bytes from serial ringbuffer */
            while (rt_device_read(_serial_dev, 0, &ch, 1) == 1)
            {
                if (frame_len < sizeof(_rx_frame_buf))
                {
                    _rx_frame_buf[frame_len++] = ch;
                }
                else
                {
                    /* Deliver a full chunk and continue; upper layers assemble across callbacks. */
                    rs232_dispatch_chunk(frame_len);
                    frame_len = 0U;
                    _rx_frame_buf[frame_len++] = ch;
                }
            }

            /* Wait for next incoming byte with idle timeout (20ms) */
            if (rt_sem_take(&_sem_rx, rt_tick_from_millisecond(RS232_IDLE_TIMEOUT_MS)) != RT_EOK)
            {
                /* Idle timeout expired! Inter-character gap reached -> Frame complete! */
                break;
            }
        }

        if (frame_len > 0)
        {
            rs232_dispatch_chunk(frame_len);
        }
    }
}

/**
 * @brief Initialize PC RS232 serial interface (19200 8N1, idle timeout framing)
 */
int pc_rs232_init(void)
{
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;

    if (_inited)
    {
        return RT_EOK;
    }

    LOG_I("Initializing PC RS232 interface on %s (PA0-RX, PA1-TX)...", RS232_UART_NAME);

    /* 1. Find device */
    _serial_dev = rt_device_find(RS232_UART_NAME);
    if (!_serial_dev)
    {
        LOG_E("Cannot find serial device: %s", RS232_UART_NAME);
        return -RT_ERROR;
    }

    /* 2. Apply the validated schema-3 saved baudrate, otherwise safe default. */
    cfg.baud_rate = param_get_rs232_baud();
    cfg.data_bits = DATA_BITS_8;
    cfg.stop_bits = STOP_BITS_1;
    cfg.parity    = PARITY_NONE;
    if (rt_device_control(_serial_dev, RT_DEVICE_CTRL_CONFIG, &cfg) != RT_EOK)
    {
        LOG_E("Failed to configure %s parameters", RS232_UART_NAME);
        return -RT_ERROR;
    }
    _current_baud = cfg.baud_rate;
    _last_activity_tick = rt_tick_get();

    /* 3. Open device with interrupt RX */
    if (rt_device_open(_serial_dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        LOG_E("Failed to open %s with INT_RX", RS232_UART_NAME);
        return -RT_ERROR;
    }

    /* 4. Initialize synchronization primitives */
    rt_sem_init(&_sem_rx, "s232_rx", 0, RT_IPC_FLAG_PRIO);
    rt_mutex_init(&_mutex_tx, "m232_tx", RT_IPC_FLAG_PRIO);

    /* 5. Set receive indication callback */
    rt_device_set_rx_indicate(_serial_dev, rs232_rx_ind);

    /* 6. Create receiver and idle-timeout framing thread */
    _rx_thread = rt_thread_create("rs232_rx",
                                  rs232_rx_thread_entry,
                                  RT_NULL,
                                  RS232_THREAD_STACK_SZ,
                                  RS232_THREAD_PRIO,
                                  10);
    if (!_rx_thread)
    {
        LOG_E("Failed to create rs232_rx thread");
        return -RT_ERROR;
    }
    rt_thread_startup(_rx_thread);

    _inited = RT_TRUE;
    LOG_I("PC RS232 ready: %u/8N1, driver chunk idle: %dms",
          _current_baud, RS232_IDLE_TIMEOUT_MS);

    return RT_EOK;
}
INIT_ENV_EXPORT(pc_rs232_init);

/**
 * @brief Register upper-layer frame receive callback
 */
void pc_rs232_set_rx_handler(pc_rs232_rx_handler_t handler)
{
    _rx_handler = handler;
}

/**
 * @brief Send raw buffer to PC over RS232
 */
rt_size_t pc_rs232_send(const void *buf, rt_size_t len)
{
    rt_size_t sent = 0;

    if (!_inited || !_serial_dev || !buf || len == 0)
    {
        return 0;
    }

    if (rt_mutex_take(&_mutex_tx, RT_WAITING_FOREVER) == RT_EOK)
    {
        sent = rt_device_write(_serial_dev, 0, buf, len);
        if (sent > 0)
        {
            _last_activity_tick = rt_tick_get();
            _stat_tx_frames++;
            _stat_tx_bytes += sent;
        }
        rt_mutex_release(&_mutex_tx);
    }

    return sent;
}

/**
 * @brief Send null-terminated string to PC over RS232
 */
rt_size_t pc_rs232_send_str(const char *str)
{
    if (!str)
    {
        return 0;
    }
    return pc_rs232_send(str, rt_strlen(str));
}

/**
 * @brief Set RS232 baudrate dynamically
 */
rt_err_t pc_rs232_set_baudrate(rt_uint32_t baud_rate)
{
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;

    if (!_inited || !_serial_dev)
    {
        return -RT_ERROR;
    }

    cfg.baud_rate = baud_rate;
    cfg.data_bits = DATA_BITS_8;
    cfg.stop_bits = STOP_BITS_1;
    cfg.parity    = PARITY_NONE;

    if (rt_device_control(_serial_dev, RT_DEVICE_CTRL_CONFIG, &cfg) == RT_EOK)
    {
        _current_baud = baud_rate;
        LOG_I("PC RS232 baudrate updated to: %u bps", baud_rate);
        return RT_EOK;
    }

    LOG_E("Failed to update baudrate to %u bps", baud_rate);
    return -RT_ERROR;
}

rt_uint32_t pc_rs232_get_baudrate(void)
{
    return _current_baud;
}

rt_bool_t pc_rs232_is_idle(rt_uint32_t quiet_ms)
{
    return ((rt_tick_get() - _last_activity_tick) >=
            rt_tick_from_millisecond(quiet_ms)) ? RT_TRUE : RT_FALSE;
}

void pc_rs232_set_loopback(rt_bool_t enable)
{
    _loopback_enabled = enable;
    LOG_I("PC RS232 loopback echo mode: %s", enable ? "ENABLED" : "DISABLED");
}

/* ==================== FinSH / MSH Commands ==================== */

#if defined(RT_USING_FINSH)

static int rs232_status(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    rt_kprintf("\r\n+-------------------------------------------------------------+\r\n");
    rt_kprintf("|              PC RS232 Serial Port (uart1) Status            |\r\n");
    rt_kprintf("+-----------------------------------+-------------------------+\r\n");
    rt_kprintf("| Parameter / Item                  | Current Value           |\r\n");
    rt_kprintf("+-----------------------------------+-------------------------+\r\n");
    rt_kprintf("| Device Name                       | %-23s |\r\n", RS232_UART_NAME);
    rt_kprintf("| Hardware Pins                     | PA0(RX), PA1(TX)        |\r\n");
    rt_kprintf("| Baudrate & Format                 | %u bps, 8N1        |\r\n", _current_baud);
    rt_kprintf("| Framing Mechanism                 | Inter-char Idle (%d ms)  |\r\n", RS232_IDLE_TIMEOUT_MS);
    rt_kprintf("| Loopback Echo Test Mode           | %-23s |\r\n", _loopback_enabled ? "ENABLED" : "DISABLED");
    rt_kprintf("| Rx Frames Count                   | %-23u |\r\n", _stat_rx_frames);
    rt_kprintf("| Rx Bytes Count                    | %-23u |\r\n", _stat_rx_bytes);
    rt_kprintf("| Tx Frames Count                   | %-23u |\r\n", _stat_tx_frames);
    rt_kprintf("| Tx Bytes Count                    | %-23u |\r\n", _stat_tx_bytes);
    rt_kprintf("+-----------------------------------+-------------------------+\r\n\r\n");

    return 0;
}
MSH_CMD_EXPORT(rs232_status, display PC RS232 uart1 status and statistics);

static int rs232_send(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: rs232_send <message_string>\r\n");
        return -1;
    }

    rt_size_t len = rt_strlen(argv[1]);
    rt_size_t sent = pc_rs232_send(argv[1], len);
    /* Also send CRLF */
    pc_rs232_send("\r\n", 2);

    rt_kprintf("[RS232] Sent %d bytes to PC: \"%s\"\r\n", sent, argv[1]);
    return 0;
}
MSH_CMD_EXPORT(rs232_send, send a test string to PC over RS232 uart1);

static int rs232_baud(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: rs232_baud <9600|19200|38400|57600|115200>\r\n");
        return -1;
    }

    rt_uint32_t baud = (rt_uint32_t)atoi(argv[1]);
    if (baud == 0)
    {
        rt_kprintf("Invalid baudrate: %s\r\n", argv[1]);
        return -1;
    }

    if (pc_rs232_set_baudrate(baud) == RT_EOK)
    {
        rt_kprintf("[RS232] Baudrate set to %u bps successfully\r\n", baud);
    }
    else
    {
        rt_kprintf("[RS232] Failed to set baudrate to %u bps\r\n", baud);
    }

    return 0;
}
MSH_CMD_EXPORT(rs232_baud, dynamically set PC RS232 uart1 baudrate);

static int rs232_echo(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: rs232_echo <on|off>\r\n");
        return -1;
    }

    if (rt_strcmp(argv[1], "on") == 0)
    {
        pc_rs232_set_loopback(RT_TRUE);
        rt_kprintf("[RS232] Loopback echo mode enabled. PC data will be reflected back.\r\n");
    }
    else if (rt_strcmp(argv[1], "off") == 0)
    {
        pc_rs232_set_loopback(RT_FALSE);
        rt_kprintf("[RS232] Loopback echo mode disabled.\r\n");
    }
    else
    {
        rt_kprintf("Unknown parameter: %s, use 'on' or 'off'\r\n", argv[1]);
        return -1;
    }

    return 0;
}
MSH_CMD_EXPORT(rs232_echo, toggle PC RS232 loopback echo mode <on|off>);

#endif /* RT_USING_FINSH */

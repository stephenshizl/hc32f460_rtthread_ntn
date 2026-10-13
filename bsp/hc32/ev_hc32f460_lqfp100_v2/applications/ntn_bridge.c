/*
 * Copyright (C) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-13     Antigravity  First version: PC <-> MCU <-> NTN transparent bridge
 */

#include "ntn_bridge.h"

#define DBG_TAG "ntn.bridge"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define BRIDGE_BUF_SIZE         256
#define BRIDGE_THREAD_STACK_SZ  2048
#define BRIDGE_THREAD_PRIORITY  10

/* Device descriptors */
static rt_device_t dev_pc  = RT_NULL;
static rt_device_t dev_ntn = RT_NULL;

/* Semaphores for notification */
static struct rt_semaphore sem_pc_rx;
static struct rt_semaphore sem_ntn_rx;

/* Statistics counters */
static volatile rt_uint32_t stat_pc_rx_bytes  = 0;
static volatile rt_uint32_t stat_pc_tx_bytes  = 0;
static volatile rt_uint32_t stat_ntn_rx_bytes = 0;
static volatile rt_uint32_t stat_ntn_tx_bytes = 0;

/* Status flags */
static rt_bool_t bridge_running = RT_FALSE;

#include "hw_power.h"

/**
 * @brief  Initialize NTN hardware control GPIOs
 */
void ntn_hw_init(void)
{
    hw_power_init();
    /* Wait for module UART ready */
    rt_thread_mdelay(500);
}

/**
 * @brief  Control NTN module power
 */
void ntn_power_control(rt_bool_t on)
{
    hw_power_ntn_set(on);
}

/**
 * @brief  Hard reset the NTN module
 */
void ntn_reset(void)
{
    hw_reset_ntn(200);
}

/**
 * @brief  Read NTN status pin level
 */
int ntn_get_status_pin(void)
{
    return hw_ntn_status_get();
}

/**
 * @brief  UART1 (PC) RX callback
 */
static rt_err_t uart_pc_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&sem_pc_rx);
    return RT_EOK;
}

/**
 * @brief  UART3 (NTN) RX callback
 */
static rt_err_t uart_ntn_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&sem_ntn_rx);
    return RT_EOK;
}

/**
 * @brief  PC to NTN forward thread: reads from UART1 (PC) and writes to UART3 (NTN)
 */
static void pc_to_ntn_entry(void *parameter)
{
    static rt_uint8_t rx_buf[BRIDGE_BUF_SIZE];
    rt_size_t rx_len;

    while (bridge_running)
    {
        rx_len = rt_device_read(dev_pc, 0, rx_buf, sizeof(rx_buf));
        if (rx_len > 0)
        {
            rt_device_write(dev_ntn, 0, rx_buf, rx_len);
            stat_pc_rx_bytes  += rx_len;
            stat_ntn_tx_bytes += rx_len;
        }
        else
        {
            /* Wait for incoming data on PC UART */
            rt_sem_take(&sem_pc_rx, RT_WAITING_FOREVER);
        }
    }
}

/**
 * @brief  NTN to PC forward thread: reads from UART3 (NTN) and writes to UART1 (PC)
 */
static void ntn_to_pc_entry(void *parameter)
{
    static rt_uint8_t rx_buf[BRIDGE_BUF_SIZE];
    rt_size_t rx_len;

    while (bridge_running)
    {
        rx_len = rt_device_read(dev_ntn, 0, rx_buf, sizeof(rx_buf));
        if (rx_len > 0)
        {
            rt_device_write(dev_pc, 0, rx_buf, rx_len);
            stat_ntn_rx_bytes += rx_len;
            stat_pc_tx_bytes  += rx_len;
        }
        else
        {
            /* Wait for incoming data on NTN UART */
            rt_sem_take(&sem_ntn_rx, RT_WAITING_FOREVER);
        }
    }
}

/**
 * @brief  Initialize transparent bridge between PC (uart1) and NTN (uart3)
 */
int ntn_bridge_init(void)
{
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;
    rt_thread_t th_pc;
    rt_thread_t th_ntn;

    /* Initialize hardware pins first */
    ntn_hw_init();

    /* Find PC UART device */
    dev_pc = rt_device_find(NTN_PC_UART_NAME);
    if (!dev_pc)
    {
        LOG_E("Cannot find device: %s", NTN_PC_UART_NAME);
        return -RT_ERROR;
    }

    /* Find NTN UART device */
    dev_ntn = rt_device_find(NTN_MODEM_UART_NAME);
    if (!dev_ntn)
    {
        LOG_E("Cannot find device: %s", NTN_MODEM_UART_NAME);
        return -RT_ERROR;
    }

    /* Configure PC UART (115200 8N1) */
    cfg.baud_rate = BAUD_RATE_115200;
    cfg.data_bits = DATA_BITS_8;
    cfg.stop_bits = STOP_BITS_1;
    cfg.parity    = PARITY_NONE;
    rt_device_control(dev_pc, RT_DEVICE_CTRL_CONFIG, &cfg);

    /* Configure NTN UART (115200 8N1) */
    rt_device_control(dev_ntn, RT_DEVICE_CTRL_CONFIG, &cfg);

    /* Open PC UART with interrupt RX */
    if (rt_device_open(dev_pc, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        LOG_E("Failed to open %s", NTN_PC_UART_NAME);
        return -RT_ERROR;
    }

    /* Open NTN UART with interrupt RX */
    if (rt_device_open(dev_ntn, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        LOG_E("Failed to open %s", NTN_MODEM_UART_NAME);
        rt_device_close(dev_pc);
        return -RT_ERROR;
    }

    /* Initialize semaphores */
    rt_sem_init(&sem_pc_rx, "sem_pcrx", 0, RT_IPC_FLAG_PRIO);
    rt_sem_init(&sem_ntn_rx, "sem_ntrx", 0, RT_IPC_FLAG_PRIO);

    /* Set receive callback handlers */
    rt_device_set_rx_indicate(dev_pc, uart_pc_rx_ind);
    rt_device_set_rx_indicate(dev_ntn, uart_ntn_rx_ind);

    bridge_running = RT_TRUE;

    /* Create PC -> NTN forwarding thread */
    th_pc = rt_thread_create("pc2ntn",
                             pc_to_ntn_entry,
                             RT_NULL,
                             BRIDGE_THREAD_STACK_SZ,
                             BRIDGE_THREAD_PRIORITY,
                             10);
    if (th_pc)
    {
        rt_thread_startup(th_pc);
    }
    else
    {
        LOG_E("Failed to create pc2ntn thread");
        return -RT_ERROR;
    }

    /* Create NTN -> PC forwarding thread */
    th_ntn = rt_thread_create("ntn2pc",
                              ntn_to_pc_entry,
                              RT_NULL,
                              BRIDGE_THREAD_STACK_SZ,
                              BRIDGE_THREAD_PRIORITY,
                              10);
    if (th_ntn)
    {
        rt_thread_startup(th_ntn);
    }
    else
    {
        LOG_E("Failed to create ntn2pc thread");
        return -RT_ERROR;
    }

    LOG_I("=================================================");
    LOG_I("  PC <-> MCU <-> NTN Transparent Bridge Started  ");
    LOG_I("  PC  Port : %s (PA0-RX, PA1-TX, 115200 8N1)   ", NTN_PC_UART_NAME);
    LOG_I("  NTN Port : %s (PB4-TX, PB5-RX, 115200 8N1)   ", NTN_MODEM_UART_NAME);
    LOG_I("=================================================");

    return RT_EOK;
}

#ifdef RT_USING_FINSH
#include <finsh.h>

static void ntn_status(void)
{
    rt_kprintf("\n[NTN Bridge Status]\n");
    rt_kprintf("  Bridge State  : %s\n", bridge_running ? "RUNNING" : "STOPPED");
    rt_kprintf("  PC  Port      : %s (115200 8N1)\n", NTN_PC_UART_NAME);
    rt_kprintf("  NTN Port      : %s (115200 8N1)\n", NTN_MODEM_UART_NAME);
    rt_kprintf("  NTN PWR_EN    : %d (PB15)\n", rt_pin_read(NTN_PWR_EN_PIN));
    rt_kprintf("  PA_PWREN      : %d (PB0)\n", rt_pin_read(PA_PWREN_PIN));
    rt_kprintf("  NTN RST       : %d (PA8)\n", rt_pin_read(NTN_RST_PIN));
    rt_kprintf("  NTN WAKEUP    : %d (PA15)\n", rt_pin_read(NTN_WAKEUP_PIN));
    rt_kprintf("  NTN BOOT      : %d (PA10)\n", rt_pin_read(NTN_BOOT_PIN));
    rt_kprintf("  NTN STATUS Pin: %d (PB3)\n", rt_pin_read(NTN_STATUS_PIN));
    rt_kprintf("  Statistics:\n");
    rt_kprintf("    PC  RX: %lu bytes -> NTN TX: %lu bytes\n", stat_pc_rx_bytes, stat_ntn_tx_bytes);
    rt_kprintf("    NTN RX: %lu bytes -> PC  TX: %lu bytes\n\n", stat_ntn_rx_bytes, stat_pc_tx_bytes);
}
MSH_CMD_EXPORT(ntn_status, show NTN transparent bridge status and counters);

static void ntn_power_cmd(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: ntn_power <on|off>\n");
        return;
    }
    if (rt_strcmp(argv[1], "on") == 0)
    {
        ntn_power_control(RT_TRUE);
    }
    else if (rt_strcmp(argv[1], "off") == 0)
    {
        ntn_power_control(RT_FALSE);
    }
    else
    {
        rt_kprintf("Invalid arg: %s, use on or off\n", argv[1]);
    }
}
MSH_CMD_EXPORT_ALIAS(ntn_power_cmd, ntn_power, control NTN power supply);

static void ntn_reset_cmd(void)
{
    ntn_reset();
}
MSH_CMD_EXPORT_ALIAS(ntn_reset_cmd, ntn_reset, reset NTN module);

#endif /* RT_USING_FINSH */

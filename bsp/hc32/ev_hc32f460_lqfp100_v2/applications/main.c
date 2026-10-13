/*
 * Copyright (C) 2022-2026, Xiaohua Semiconductor Co., Ltd. / NTN Project.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-28     CDT          first version
 * 2026-10-13     Stephen      Integrate PC RS232 (uart1, 19200/8N1, idle framing)
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "hw_power.h"
#include "pc_rs232.h"

int main(void)
{
    rt_kprintf("\r\n[SYS] NTN System Main Thread Started...\r\n");

    /* 1. Initialize hardware power rails and peripheral GPIOs */
    hw_power_init();

    /* 2. Initialize PC RS232 communication interface (uart1, 19200/8N1) */
    pc_rs232_init();

    /* Send boot banner to PC RS232 */
    pc_rs232_send_str("\r\n======================================================\r\n");
    pc_rs232_send_str("  Iridium NTN Smart Terminal - PC RS232 Port Ready    \r\n");
    pc_rs232_send_str("  UART: uart1 (PA0-RX, PA1-TX) @ 19200 bps 8N1       \r\n");
    pc_rs232_send_str("======================================================\r\n\r\n");

    while (1)
    {
        /* Reflect NTN network status pin on NET_OK LED */
        hw_led_net_set(hw_ntn_status_get() ? RT_TRUE : RT_FALSE);

        /* Power LED heartbeat (500ms toggle) */
        hw_led_power_set(RT_TRUE);
        rt_thread_mdelay(500);
        hw_led_power_set(RT_FALSE);
        rt_thread_mdelay(500);
    }
}

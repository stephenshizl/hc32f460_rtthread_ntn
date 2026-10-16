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

int main(void)
{
    rt_kprintf("\r\n[SYS] NTN System Main Thread Started...\r\n");

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

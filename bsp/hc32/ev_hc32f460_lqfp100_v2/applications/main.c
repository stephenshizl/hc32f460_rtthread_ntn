/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-28     CDT          first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "ntn_bridge.h"

/* Board Status LED pins */
#define PIN_LED_POWER_OK    GET_PIN(A, 6)   /* POWER_OK: PA6 */
#define PIN_LED_NET_OK      GET_PIN(A, 5)   /* NET_OK: PA5 */

int main(void)
{
    /* Initialize board status LEDs */
    rt_pin_mode(PIN_LED_POWER_OK, PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_LED_NET_OK, PIN_MODE_OUTPUT);

    rt_pin_write(PIN_LED_POWER_OK, PIN_HIGH);
    rt_pin_write(PIN_LED_NET_OK, PIN_LOW);

    /* Initialize PC <-> MCU <-> NTN transparent pass-through bridge */
    ntn_bridge_init();

    while (1)
    {
        /* Reflect NTN network status pin on NET_OK LED */
        if (ntn_get_status_pin())
        {
            rt_pin_write(PIN_LED_NET_OK, PIN_HIGH);
        }
        else
        {
            rt_pin_write(PIN_LED_NET_OK, PIN_LOW);
        }

        /* Power LED heartbeat */
        rt_pin_write(PIN_LED_POWER_OK, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(PIN_LED_POWER_OK, PIN_LOW);
        rt_thread_mdelay(500);
    }
}


/*
 * Copyright (C) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-13     Antigravity  First version: PC <-> MCU <-> NTN transparent bridge
 */

#ifndef __NTN_BRIDGE_H__
#define __NTN_BRIDGE_H__

#include <rtthread.h>
#include <rtdevice.h>
#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Device names */
#define NTN_PC_UART_NAME        "uart1"     /* PC RS232: PA0(RX), PA1(TX) */
#define NTN_MODEM_UART_NAME     "uart3"     /* NTN ST1000: PB4(TX), PB5(RX) */

/* Hardware control API */
void ntn_hw_init(void);
void ntn_power_control(rt_bool_t on);
void ntn_reset(void);
int ntn_get_status_pin(void);

/* Bridge API */
int ntn_bridge_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __NTN_BRIDGE_H__ */

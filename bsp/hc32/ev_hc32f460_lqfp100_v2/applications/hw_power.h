/*
 * Copyright (c) 2026, youyoutech.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-13     Stephen      First version: Hardware Power & Reset Manager
 */

#ifndef __HW_POWER_H__
#define __HW_POWER_H__

#include <rtthread.h>
#include <rtdevice.h>
#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize all hardware power control, reset, and status GPIOs
 */
int hw_power_init(void);

/**
 * @brief Control NTN modem power rail (PB15) & PA power rail (PB0) with sequence
 * @param on RT_TRUE to power up with proper sequencing, RT_FALSE to power down
 */
void hw_power_ntn_set(rt_bool_t on);

/**
 * @brief Control GNSS/RNSS power rail (PH2) and reset (PC13)
 * @param on RT_TRUE to enable, RT_FALSE to disable
 */
void hw_power_rnss_set(rt_bool_t on);

/**
 * @brief Control Bluetooth BLE power rail (PB9)
 * @param on RT_TRUE to enable, RT_FALSE to disable
 */
void hw_power_bt_set(rt_bool_t on);

/**
 * @brief Pulse hardware reset to NTN modem (PA8)
 * @param pulse_ms Pulse duration in milliseconds (typically 50-100ms)
 */
void hw_reset_ntn(rt_uint32_t pulse_ms);

/**
 * @brief Pulse hardware reset to GNSS module (PC13)
 * @param pulse_ms Pulse duration in milliseconds
 */
void hw_reset_rnss(rt_uint32_t pulse_ms);

/**
 * @brief Get NTN status pin value (PB3)
 * @return 0 (Low) or 1 (High)
 */
int hw_ntn_status_get(void);

/**
 * @brief Control NTN modem wakeup pin level (PA15)
 * @param high RT_TRUE for High, RT_FALSE for Low
 */
void hw_ntn_wakeup_set(rt_bool_t high);

/**
 * @brief Pulse NTN modem wakeup pin (PA15)
 * @param pulse_ms Duration of pulse in ms
 * @note Establishes the opposite level for 10 ms before generating the pulse.
 * @param to_low RT_TRUE: pulse Low (High -> Low -> High); RT_FALSE: pulse High (Low -> High -> Low)
 */
void hw_ntn_wakeup_pulse(rt_uint32_t pulse_ms, rt_bool_t to_low);

/**
 * @brief Control Power OK LED (PA6)
 */
void hw_led_power_set(rt_bool_t on);

/**
 * @brief Control Net OK LED (PA5)
 */
void hw_led_net_set(rt_bool_t on);

/**
 * @brief Query current power state of each sub-module
 */
rt_bool_t hw_power_ntn_get(void);
rt_bool_t hw_power_rnss_get(void);
rt_bool_t hw_power_bt_get(void);

#ifdef __cplusplus
}
#endif

#endif /* __HW_POWER_H__ */

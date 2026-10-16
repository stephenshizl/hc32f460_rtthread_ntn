/*
 * Copyright (c) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-13     Stephen      First version: Hardware Power & Reset Manager
 */

#include "hw_power.h"
#include <stdlib.h>

#define DBG_TAG "hw.pwr"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static rt_bool_t _ntn_power_state  = RT_FALSE;
static rt_bool_t _rnss_power_state = RT_FALSE;
static rt_bool_t _bt_power_state   = RT_FALSE;
static rt_bool_t _pwr_inited       = RT_FALSE;

/**
 * @brief Initialize all hardware power control, reset, and status GPIOs
 */
int hw_power_init(void)
{
    if (_pwr_inited)
    {
        return RT_EOK;
    }

    LOG_I("Initializing hardware power rails and control GPIOs...");

    /* 1. Board Status LEDs: PA6(POWER_OK), PA5(NET_OK) */
    rt_pin_mode(PIN_LED_POWER_OK, PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_LED_NET_OK, PIN_MODE_OUTPUT);
    rt_pin_write(PIN_LED_POWER_OK, PIN_HIGH);   /* Turn on Power LED */
    rt_pin_write(PIN_LED_NET_OK, PIN_LOW);      /* Turn off Net LED initially */

    /* 2. NTN Modem Control GPIOs:
     * - PB15: NTN Core Power Switch (PMOS)
     * - PB0:  RF PA 4.16V DCDC Enable
     * - PA8:  Hardware Reset (Active High turns on NPN pulling ST1000 RESETN Low)
     * - PA15: Wakeup (High = active)
     * - PA10: Boot mode (Low = normal, High = bootloader)
     * - PB3:  Status input
     */
    rt_pin_mode(NTN_BOOT_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(NTN_BOOT_PIN, PIN_LOW);

    rt_pin_mode(NTN_WAKEUP_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(NTN_WAKEUP_PIN, PIN_HIGH);

    rt_pin_mode(NTN_STATUS_PIN, PIN_MODE_INPUT);

    rt_pin_mode(NTN_RST_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(NTN_RST_PIN, PIN_HIGH);        /* Keep in reset initially */

    rt_pin_mode(NTN_PWR_EN_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(NTN_PWR_EN_PIN, PIN_LOW);      /* Core power default off */

    rt_pin_mode(PA_PWREN_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(PA_PWREN_PIN, PIN_LOW);        /* RF PA default off */

    /* 3. GNSS / 北斗 Control GPIOs:
     * - PH2:  SGM2032 3.3V LDO Enable
     * - PC13: Hardware Reset (Low active)
     */
    rt_pin_mode(RNSS_RST_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(RNSS_RST_PIN, PIN_LOW);        /* Keep in reset initially */

    rt_pin_mode(RNSS_PWR_EN_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(RNSS_PWR_EN_PIN, PIN_LOW);     /* Power default off */

    /* 4. BLE Bluetooth Control GPIOs:
     * - PB9: SGM2032 3.3V LDO Enable
     */
    rt_pin_mode(BT_PWR_EN_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(BT_PWR_EN_PIN, PIN_LOW);       /* Power default off */

    _pwr_inited = RT_TRUE;
    LOG_I("Hardware GPIO initialization completed.");

    /* Default boot behavior: Power on NTN, GNSS, and BT modules sequentially */
    hw_power_ntn_set(RT_TRUE);
    hw_power_rnss_set(RT_TRUE);
    hw_power_bt_set(RT_TRUE);

    return RT_EOK;
}
/* GPIO safe states and power rails must be ready before UART services start. */
INIT_PREV_EXPORT(hw_power_init);

/**
 * @brief Control NTN modem power rail (PB15) & PA power rail (PB0) with sequence
 */
void hw_power_ntn_set(rt_bool_t on)
{
    if (on)
    {
        LOG_I("[NTN] Powering ON modem and RF PA...");

        /* 1. Ensure normal run mode and wakeup high */
        rt_pin_write(NTN_BOOT_PIN, PIN_LOW);
        rt_pin_write(NTN_WAKEUP_PIN, PIN_HIGH);

        /* 2. Hold reset asserted while power is ramping up */
        rt_pin_write(NTN_RST_PIN, PIN_HIGH);

        /* 3. Enable core power PMOS and PA DCDC */
        rt_pin_write(NTN_PWR_EN_PIN, PIN_HIGH);
        rt_pin_write(PA_PWREN_PIN, PIN_HIGH);

        /* 4. Wait 100ms for power rails (3.3V and 4.16V) to stabilize */
        rt_thread_mdelay(100);

        /* 5. Release reset: set PA8 LOW so ST1000 RESETN pulls high */
        rt_pin_write(NTN_RST_PIN, PIN_LOW);
        _ntn_power_state = RT_TRUE;

        LOG_I("[NTN] Power ON sequence completed, modem is booting.");
    }
    else
    {
        LOG_I("[NTN] Powering OFF modem and RF PA...");

        /* Assert reset */
        rt_pin_write(NTN_RST_PIN, PIN_HIGH);

        /* Turn off PA and core power */
        rt_pin_write(PA_PWREN_PIN, PIN_LOW);
        rt_pin_write(NTN_PWR_EN_PIN, PIN_LOW);
        rt_pin_write(NTN_WAKEUP_PIN, PIN_LOW);

        _ntn_power_state = RT_FALSE;
        LOG_I("[NTN] Power OFF completed.");
    }
}

/**
 * @brief Control GNSS/RNSS power rail (PH2) and reset (PC13)
 */
void hw_power_rnss_set(rt_bool_t on)
{
    if (on)
    {
        LOG_I("[RNSS] Powering ON GNSS module (PH2 -> High)...");
        rt_pin_write(RNSS_RST_PIN, PIN_LOW);        /* Keep in reset */
        rt_pin_write(RNSS_PWR_EN_PIN, PIN_HIGH);   /* Enable LDO 3.3V */
        rt_thread_mdelay(20);                       /* LDO stabilization */
        rt_pin_write(RNSS_RST_PIN, PIN_HIGH);       /* Release reset */
        _rnss_power_state = RT_TRUE;
        LOG_I("[RNSS] GNSS power ON completed.");
    }
    else
    {
        LOG_I("[RNSS] Powering OFF GNSS module...");
        rt_pin_write(RNSS_RST_PIN, PIN_LOW);
        rt_pin_write(RNSS_PWR_EN_PIN, PIN_LOW);
        _rnss_power_state = RT_FALSE;
        LOG_I("[RNSS] GNSS power OFF completed.");
    }
}

/**
 * @brief Control Bluetooth BLE power rail (PB9)
 */
void hw_power_bt_set(rt_bool_t on)
{
    if (on)
    {
        LOG_I("[BT] Powering ON Bluetooth BLE module (PB9 -> High)...");
        rt_pin_write(BT_PWR_EN_PIN, PIN_HIGH);      /* Enable LDO 3.3V */
        _bt_power_state = RT_TRUE;
        LOG_I("[BT] Bluetooth power ON completed.");
    }
    else
    {
        LOG_I("[BT] Powering OFF Bluetooth BLE module...");
        rt_pin_write(BT_PWR_EN_PIN, PIN_LOW);
        _bt_power_state = RT_FALSE;
        LOG_I("[BT] Bluetooth power OFF completed.");
    }
}

/**
 * @brief Pulse hardware reset to NTN modem (PA8)
 */
void hw_reset_ntn(rt_uint32_t pulse_ms)
{
    if (pulse_ms == 0)
    {
        pulse_ms = 100;
    }

    LOG_I("[NTN] Asserting hardware reset pulse (%d ms)...", pulse_ms);
    rt_pin_write(NTN_RST_PIN, PIN_HIGH);
    rt_thread_mdelay(pulse_ms);
    rt_pin_write(NTN_RST_PIN, PIN_LOW);
    LOG_I("[NTN] Hardware reset released.");
}

/**
 * @brief Pulse hardware reset to GNSS module (PC13)
 */
void hw_reset_rnss(rt_uint32_t pulse_ms)
{
    if (pulse_ms == 0)
    {
        pulse_ms = 50;
    }

    LOG_I("[RNSS] Asserting GNSS reset pulse (%d ms)...", pulse_ms);
    rt_pin_write(RNSS_RST_PIN, PIN_LOW);
    rt_thread_mdelay(pulse_ms);
    rt_pin_write(RNSS_RST_PIN, PIN_HIGH);
    LOG_I("[RNSS] GNSS reset released.");
}

/**
 * @brief Get NTN status pin value (PB3)
 */
int hw_ntn_status_get(void)
{
    return rt_pin_read(NTN_STATUS_PIN);
}

/**
 * @brief Control Power OK LED (PA6)
 */
void hw_led_power_set(rt_bool_t on)
{
    rt_pin_write(PIN_LED_POWER_OK, on ? PIN_HIGH : PIN_LOW);
}

/**
 * @brief Control Net OK LED (PA5)
 */
void hw_led_net_set(rt_bool_t on)
{
    rt_pin_write(PIN_LED_NET_OK, on ? PIN_HIGH : PIN_LOW);
}

rt_bool_t hw_power_ntn_get(void)
{
    return _ntn_power_state;
}

rt_bool_t hw_power_rnss_get(void)
{
    return _rnss_power_state;
}

rt_bool_t hw_power_bt_get(void)
{
    return _bt_power_state;
}

/* ==================== FinSH / MSH Commands ==================== */

#if defined(RT_USING_FINSH)

static int power_status(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    rt_kprintf("\r\n+-------------------------------------------------------------+\r\n");
    rt_kprintf("|         Iridium NTN Terminal Hardware Power & GPIO Status   |\r\n");
    rt_kprintf("+-----------------------------------+---------+---------------+\r\n");
    rt_kprintf("| Peripheral Rail / Function        | Pin     | Current State |\r\n");
    rt_kprintf("+-----------------------------------+---------+---------------+\r\n");
    rt_kprintf("| NTN Modem Core Power (PMOS)       | PB15    | %-13s |\r\n", _ntn_power_state ? "ON (High)" : "OFF (Low)");
    rt_kprintf("| NTN RF PA Power (DCDC 4.16V)      | PB0     | %-13s |\r\n", _ntn_power_state ? "ON (High)" : "OFF (Low)");
    rt_kprintf("| NTN Reset Pin (Q3 NPN)            | PA8     | %-13s |\r\n", rt_pin_read(NTN_RST_PIN) ? "ASSERT (High)" : "RUN (Low)");
    rt_kprintf("| NTN Wakeup Pin                    | PA15    | %-13s |\r\n", rt_pin_read(NTN_WAKEUP_PIN) ? "HIGH (Active)" : "LOW (Sleep)");
    rt_kprintf("| NTN Boot Mode Pin                 | PA10    | %-13s |\r\n", rt_pin_read(NTN_BOOT_PIN) ? "BOOT (High)" : "NORMAL (Low)");
    rt_kprintf("| NTN Status Feedback Pin           | PB3     | %-13s |\r\n", hw_ntn_status_get() ? "HIGH" : "LOW");
    rt_kprintf("+-----------------------------------+---------+---------------+\r\n");
    rt_kprintf("| GNSS / RNSS Power (LDO 3.3V)      | PH2     | %-13s |\r\n", _rnss_power_state ? "ON (High)" : "OFF (Low)");
    rt_kprintf("| GNSS / RNSS Reset Pin             | PC13    | %-13s |\r\n", rt_pin_read(RNSS_RST_PIN) ? "RUN (High)" : "RESET (Low)");
    rt_kprintf("+-----------------------------------+---------+---------------+\r\n");
    rt_kprintf("| BLE Bluetooth Power (LDO 3.3V)    | PB9     | %-13s |\r\n", _bt_power_state ? "ON (High)" : "OFF (Low)");
    rt_kprintf("+-----------------------------------+---------+---------------+\r\n");
    rt_kprintf("| Status LED: POWER_OK              | PA6     | %-13s |\r\n", rt_pin_read(PIN_LED_POWER_OK) ? "ON (High)" : "OFF (Low)");
    rt_kprintf("| Status LED: NET_OK                | PA5     | %-13s |\r\n", rt_pin_read(PIN_LED_NET_OK) ? "ON (High)" : "OFF (Low)");
    rt_kprintf("+-----------------------------------+---------+---------------+\r\n\r\n");

    return 0;
}
MSH_CMD_EXPORT(power_status, show hardware power rails and pin status);

static int power_ntn(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: power_ntn <on|off>\r\n");
        return -1;
    }

    if (rt_strcmp(argv[1], "on") == 0)
    {
        hw_power_ntn_set(RT_TRUE);
    }
    else if (rt_strcmp(argv[1], "off") == 0)
    {
        hw_power_ntn_set(RT_FALSE);
    }
    else
    {
        rt_kprintf("Unknown parameter: %s, use 'on' or 'off'\r\n", argv[1]);
        return -1;
    }

    return 0;
}
MSH_CMD_EXPORT(power_ntn, control NTN modem power rail <on|off>);

static int power_rnss(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: power_rnss <on|off>\r\n");
        return -1;
    }

    if (rt_strcmp(argv[1], "on") == 0)
    {
        hw_power_rnss_set(RT_TRUE);
    }
    else if (rt_strcmp(argv[1], "off") == 0)
    {
        hw_power_rnss_set(RT_FALSE);
    }
    else
    {
        rt_kprintf("Unknown parameter: %s, use 'on' or 'off'\r\n", argv[1]);
        return -1;
    }

    return 0;
}
MSH_CMD_EXPORT(power_rnss, control GNSS/RNSS power rail <on|off>);

static int power_bt(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: power_bt <on|off>\r\n");
        return -1;
    }

    if (rt_strcmp(argv[1], "on") == 0)
    {
        hw_power_bt_set(RT_TRUE);
    }
    else if (rt_strcmp(argv[1], "off") == 0)
    {
        hw_power_bt_set(RT_FALSE);
    }
    else
    {
        rt_kprintf("Unknown parameter: %s, use 'on' or 'off'\r\n", argv[1]);
        return -1;
    }

    return 0;
}
MSH_CMD_EXPORT(power_bt, control Bluetooth BLE power rail <on|off>);

static int power_reset_ntn(int argc, char **argv)
{
    rt_uint32_t ms = 100;
    if (argc >= 2)
    {
        ms = (rt_uint32_t)atoi(argv[1]);
    }

    hw_reset_ntn(ms);
    return 0;
}
MSH_CMD_EXPORT(power_reset_ntn, trigger NTN hardware reset pulse [pulse_ms]);

static int power_reset_rnss(int argc, char **argv)
{
    rt_uint32_t ms = 50;
    if (argc >= 2)
    {
        ms = (rt_uint32_t)atoi(argv[1]);
    }

    hw_reset_rnss(ms);
    return 0;
}
MSH_CMD_EXPORT(power_reset_rnss, trigger GNSS hardware reset pulse [pulse_ms]);

#endif /* RT_USING_FINSH */

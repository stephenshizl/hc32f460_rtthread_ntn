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
#include "param_manager.h"
#include "mcu_local_cmd.h"
#ifdef NTN_OTA_APP
#include "ota_app.h"
#include "ota_power.h"
#include "ota_watchdog.h"
#include "store_forward.h"
#include "task_wdt.h"
#endif

#ifdef RT_USING_FINSH
#include <finsh.h>

static int sys_reboot(int argc, char **argv)
{
    mcu_cmd_action_t action = {MCU_CMD_ACTION_REBOOT, 0U, 0U};
    RT_UNUSED(argv);
    if (argc != 1)
    {
        rt_kprintf("Usage: sys_reboot\n");
        return -RT_EINVAL;
    }
    rt_kprintf("[SYS] Rebooting MCU in 500 ms; saved NV configuration is retained.\n");
    mcu_local_cmd_apply_reboot(&action);
    return RT_EOK;
}
MSH_CMD_EXPORT(sys_reboot, Reboot MCU system in 500 ms and retain saved NV);
#endif

int main(void)
{
    char boot_log[192];
#ifdef NTN_OTA_APP
    rt_tick_t ota_started = rt_tick_get();
    bool ota_confirmed;
    ntn_ota_feed_watchdog();
    ota_confirmed = ntn_ota_is_confirmed();
#endif

    rt_kprintf("\r\n[SYS] NTN System Main Thread Started...\r\n");
    /* One boot record on the customer port, after the PC router is initialized. */
    rt_snprintf(boot_log, sizeof(boot_log),
                "\r\n[BOOT] NTN-Terminal started, RS232=%u/8N1, mode=%u, config=%s, fw=PC_AT_V4, build=%s %s\r\n",
                pc_rs232_get_baudrate(), param_get_work_mode(),
                param_has_persisted_config() ? "saved" : "RAM defaults", __DATE__, __TIME__);
    pc_rs232_send_str(boot_log);

    /* Keep POWER_OK high while the system is running. */
    hw_led_power_set(RT_TRUE);

    while (1)
    {
#ifdef NTN_OTA_APP
        if (!ota_confirmed && rt_tick_get() - ota_started >= rt_tick_from_millisecond(6000U))
        {
            if (task_wdt_is_confirmed_healthy() && store_forward_is_ready() &&
                rt_device_find(RS232_UART_NAME) && pc_rs232_get_baudrate() != 0U)
                ota_confirmed = ntn_ota_confirm();
            if (!ota_confirmed && rt_tick_get() - ota_started >= rt_tick_from_millisecond(60000U))
                rt_hw_cpu_reset();
        }
#endif
        /* Reflect NTN network status pin on NET_OK LED */
        hw_led_net_set(hw_ntn_status_get() ? RT_TRUE : RT_FALSE);

        rt_thread_mdelay(1000);
    }
}

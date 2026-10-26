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

    rt_kprintf("\r\n[SYS] NTN System Main Thread Started...\r\n");
    /* One boot record on the customer port, after the PC router is initialized. */
    rt_snprintf(boot_log, sizeof(boot_log),
                "\r\n[BOOT] NTN-Terminal started, RS232=%u/8N1, mode=%u, config=%s, fw=PC_AT_V4, build=%s %s\r\n",
                pc_rs232_get_baudrate(), param_get_work_mode(),
                param_has_persisted_config() ? "saved" : "RAM defaults", __DATE__, __TIME__);
    pc_rs232_send_str(boot_log);

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

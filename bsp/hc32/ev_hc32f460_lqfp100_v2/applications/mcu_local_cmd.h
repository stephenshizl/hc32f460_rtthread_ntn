/* SPDX-License-Identifier: Apache-2.0 */

#ifndef __MCU_LOCAL_CMD_H__
#define __MCU_LOCAL_CMD_H__

#include <rtthread.h>

typedef enum
{
    MCU_CMD_ACTION_NONE = 0,
    MCU_CMD_ACTION_APPLY_RS232_BAUD,
    MCU_CMD_ACTION_ENTER_PC_CONFIG,
    MCU_CMD_ACTION_EXIT_PC_CONFIG,
    MCU_CMD_ACTION_REBOOT,
    MCU_CMD_ACTION_OTA_BOOT
} mcu_cmd_action_type_t;

typedef struct
{
    mcu_cmd_action_type_t type;
    rt_uint32_t value;
} mcu_cmd_action_t;

/*
 * Execute one CR-stripped AT line.
 * Return RT_TRUE only when the line exactly matches a registered local command.
 */
rt_bool_t mcu_local_cmd_execute(const rt_uint8_t *line, rt_size_t line_len,
                                char *response, rt_size_t response_size,
                                mcu_cmd_action_t *action);

/* Finish an accepted baud change after flushing the old-rate response. */
void mcu_local_cmd_complete_baud(rt_uint32_t baud, rt_bool_t port_ready,
                                 char *out, rt_size_t out_size);

/* Call only after sending the command response, in thread context. */
void mcu_local_cmd_apply_reboot(const mcu_cmd_action_t *action);

#endif /* __MCU_LOCAL_CMD_H__ */

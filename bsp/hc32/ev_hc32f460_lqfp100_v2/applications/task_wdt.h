/*
 * Copyright (c) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-14     Stephen      Stage 5.1: Multi-Task Heartbeat Watchdog Framework
 */

#ifndef __TASK_WDT_H__
#define __TASK_WDT_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TASK_WDT_MAX_TASKS          8       /* Maximum registered tasks */
#define TASK_WDT_NAME_MAX_LEN       16      /* Max task identifier string length */
#define TASK_WDT_HW_TIMEOUT_DEF     10      /* Hardware WDT period in seconds (max ~10.7s) */
#define TASK_WDT_INVALID_ID         (-1)

typedef int task_wdt_id_t;

/**
 * @brief Registered task monitoring descriptor
 */
typedef struct
{
    char        name[TASK_WDT_NAME_MAX_LEN];    /* Human-readable task name */
    rt_uint32_t timeout_sec;                    /* Allowed maximum silence duration in seconds */
    rt_tick_t   last_heartbeat_tick;            /* System tick of last received heartbeat */
    rt_bool_t   is_used;                        /* Slot occupied flag */
    rt_bool_t   is_enabled;                     /* Active monitoring flag */
    rt_bool_t   is_hung;                        /* Hung/deadlock detected flag */
} task_wdt_entry_t;

/**
 * @brief Initialize Task Watchdog framework, start supervisor daemon and arm hardware WDT
 * @return RT_EOK on success, negative error code on failure
 */
int task_wdt_init(void);

/**
 * @brief Register a thread/task to be monitored by the watchdog supervisor
 * @param task_name Unique human-readable name of the task
 * @param timeout_sec Maximum acceptable silence in seconds before declaring the task hung
 * @return Positive task ID on success, TASK_WDT_INVALID_ID on failure
 */
task_wdt_id_t task_wdt_register(const char *task_name, rt_uint32_t timeout_sec);

/**
 * @brief Send a heartbeat from a monitored task to prove it is alive and responsive
 * @param id Task ID returned by task_wdt_register
 */
void task_wdt_heartbeat(task_wdt_id_t id);

/**
 * @brief Send a heartbeat by task name
 * @param task_name Name used during registration
 */
void task_wdt_heartbeat_by_name(const char *task_name);

/**
 * @brief Temporarily enable or disable watchdog monitoring for a specific task
 * (e.g. when a module is powered down or sleeping)
 * @param id Task ID
 * @param enable RT_TRUE to enable, RT_FALSE to pause monitoring
 */
void task_wdt_set_enable(task_wdt_id_t id, rt_bool_t enable);

/**
 * @brief Check if all monitored tasks are healthy
 * @return RT_TRUE if healthy, RT_FALSE if any task has hung
 */
rt_bool_t task_wdt_is_all_healthy(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_WDT_H__ */

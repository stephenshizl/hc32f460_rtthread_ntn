/*
 * Copyright (c) 2026, youyoutech.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-14     Stephen      Stage 5.1: Multi-Task Heartbeat Watchdog Framework
 */

#include "task_wdt.h"
#include <string.h>
#ifdef NTN_OTA_APP
#include "ota_watchdog.h"
#endif

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "app.wdt"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define WDT_DEV_NAME "wdt"

static task_wdt_entry_t s_tasks[TASK_WDT_MAX_TASKS];
static struct rt_mutex  s_wdt_lock;
static rt_device_t      s_hw_wdt_dev = RT_NULL;
static rt_thread_t      s_wdt_thread = RT_NULL;
static rt_bool_t        s_global_enabled = RT_TRUE;
static rt_bool_t        s_wdt_init_ok = RT_FALSE;
static rt_uint32_t      s_healthy_rounds = 0U;

task_wdt_id_t task_wdt_register(const char *task_name, rt_uint32_t timeout_sec)
{
    if (task_name == RT_NULL || timeout_sec == 0)
    {
        return TASK_WDT_INVALID_ID;
    }

    rt_mutex_take(&s_wdt_lock, RT_WAITING_FOREVER);

    /* Check if already registered */
    for (int i = 0; i < TASK_WDT_MAX_TASKS; i++)
    {
        if (s_tasks[i].is_used && rt_strcmp(s_tasks[i].name, task_name) == 0)
        {
            s_tasks[i].timeout_sec = timeout_sec;
            s_tasks[i].last_heartbeat_tick = rt_tick_get();
            s_tasks[i].is_hung = RT_FALSE;
            s_tasks[i].is_enabled = RT_TRUE;
            rt_mutex_release(&s_wdt_lock);
            LOG_D("Task '%s' updated in Watchdog registry (ID=%d, Timeout=%d s)", task_name, i, timeout_sec);
            return (task_wdt_id_t)i;
        }
    }

    /* Find empty slot */
    for (int i = 0; i < TASK_WDT_MAX_TASKS; i++)
    {
        if (!s_tasks[i].is_used)
        {
            rt_strncpy(s_tasks[i].name, task_name, TASK_WDT_NAME_MAX_LEN - 1);
            s_tasks[i].name[TASK_WDT_NAME_MAX_LEN - 1] = '\0';
            s_tasks[i].timeout_sec = timeout_sec;
            s_tasks[i].last_heartbeat_tick = rt_tick_get();
            s_tasks[i].is_used = RT_TRUE;
            s_tasks[i].is_enabled = RT_TRUE;
            s_tasks[i].is_hung = RT_FALSE;
            rt_mutex_release(&s_wdt_lock);
            LOG_I("Task '%s' registered to Watchdog (ID=%d, Timeout=%d s)", task_name, i, timeout_sec);
            return (task_wdt_id_t)i;
        }
    }

    rt_mutex_release(&s_wdt_lock);
    LOG_E("Failed to register task '%s': watchdog registry full!", task_name);
    return TASK_WDT_INVALID_ID;
}

void task_wdt_heartbeat(task_wdt_id_t id)
{
    if (id < 0 || id >= TASK_WDT_MAX_TASKS)
    {
        return;
    }

    if (s_tasks[id].is_used)
    {
        s_tasks[id].last_heartbeat_tick = rt_tick_get();
        s_tasks[id].heartbeat_count++;
        s_tasks[id].is_hung = RT_FALSE;
    }
}

void task_wdt_heartbeat_by_name(const char *task_name)
{
    if (task_name == RT_NULL || !s_wdt_init_ok)
    {
        return;
    }

    for (int i = 0; i < TASK_WDT_MAX_TASKS; i++)
    {
        if (s_tasks[i].is_used && rt_strcmp(s_tasks[i].name, task_name) == 0)
        {
            s_tasks[i].last_heartbeat_tick = rt_tick_get();
            s_tasks[i].heartbeat_count++;
            s_tasks[i].is_hung = RT_FALSE;
            break;
        }
    }
}

void task_wdt_set_enable(task_wdt_id_t id, rt_bool_t enable)
{
    if (id < 0 || id >= TASK_WDT_MAX_TASKS)
    {
        return;
    }

    rt_mutex_take(&s_wdt_lock, RT_WAITING_FOREVER);
    if (s_tasks[id].is_used)
    {
        s_tasks[id].is_enabled = enable;
        if (enable)
        {
            s_tasks[id].last_heartbeat_tick = rt_tick_get();
            s_tasks[id].is_hung = RT_FALSE;
        }
    }
    rt_mutex_release(&s_wdt_lock);
}

rt_bool_t task_wdt_is_all_healthy(void)
{
    rt_tick_t now = rt_tick_get();
    rt_bool_t all_healthy = RT_TRUE;

    for (int i = 0; i < TASK_WDT_MAX_TASKS; i++)
    {
        if (s_tasks[i].is_used && s_tasks[i].is_enabled)
        {
            rt_uint32_t elapsed_s = (now - s_tasks[i].last_heartbeat_tick) / RT_TICK_PER_SECOND;
            if (elapsed_s > s_tasks[i].timeout_sec)
            {
                s_tasks[i].is_hung = RT_TRUE;
                all_healthy = RT_FALSE;
            }
        }
    }

    return all_healthy;
}

rt_bool_t task_wdt_is_confirmed_healthy(void)
{
    static const char *const required[] = {"rs232", "ntn", "ble", "gnss"};
    rt_bool_t healthy = RT_TRUE;

    if (!s_wdt_init_ok)
        return RT_FALSE;
    rt_mutex_take(&s_wdt_lock, RT_WAITING_FOREVER);
    for (rt_size_t r = 0; r < sizeof(required) / sizeof(required[0]); ++r)
    {
        int i;
        for (i = 0; i < TASK_WDT_MAX_TASKS; ++i)
        {
            if (s_tasks[i].is_used && s_tasks[i].is_enabled &&
                s_tasks[i].heartbeat_count && rt_strcmp(s_tasks[i].name, required[r]) == 0)
                break;
        }
        if (i == TASK_WDT_MAX_TASKS)
        {
            healthy = RT_FALSE;
            break;
        }
    }
    healthy = healthy && s_global_enabled && s_healthy_rounds >= 3U && task_wdt_is_all_healthy();
    rt_mutex_release(&s_wdt_lock);
    return healthy;
}

/* Supervisor Daemon: runs at 1 Hz */
static void task_wdt_daemon_entry(void *param)
{
    LOG_I("Task Watchdog supervisor daemon started (1 Hz check cycle).");

    while (1)
    {
        rt_thread_mdelay(1000);

        if (!s_global_enabled)
        {
            continue;
        }

        rt_mutex_take(&s_wdt_lock, RT_WAITING_FOREVER);

        rt_tick_t now = rt_tick_get();
        rt_bool_t all_healthy = RT_TRUE;
        rt_uint32_t active_tasks = 0U;

        for (int i = 0; i < TASK_WDT_MAX_TASKS; i++)
        {
            if (s_tasks[i].is_used && s_tasks[i].is_enabled)
            {
                active_tasks++;
                rt_uint32_t elapsed_s = (now - s_tasks[i].last_heartbeat_tick) / RT_TICK_PER_SECOND;

                if (elapsed_s > s_tasks[i].timeout_sec)
                {
                    s_tasks[i].is_hung = RT_TRUE;
                    all_healthy = RT_FALSE;

                    LOG_E("[CRITICAL] Task '%s' is HUNG! (No heartbeat for %d s, limit %d s). Hardware WDT will reset system!",
                          s_tasks[i].name, elapsed_s, s_tasks[i].timeout_sec);
                }
            }
        }

        /* Only feed hardware WDT if ALL tasks are healthy */
        if (all_healthy && active_tasks > 0U)
        {
            if (s_healthy_rounds < 0xFFFFFFFFUL)
            {
                s_healthy_rounds++;
            }
#ifdef NTN_OTA_APP
            ntn_ota_feed_watchdog();
#endif
            if (s_hw_wdt_dev != RT_NULL)
            {
                rt_device_control(s_hw_wdt_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, RT_NULL);
            }
        }
        else
        {
            s_healthy_rounds = 0U;
            LOG_W("Suppressed hardware WDT keepalive! Awaiting hardware watchdog reset countdown...");
        }

        rt_mutex_release(&s_wdt_lock);
    }
}

int task_wdt_init(void)
{
    if (s_wdt_init_ok)
    {
        return RT_EOK;
    }

    rt_memset(s_tasks, 0, sizeof(s_tasks));
    rt_mutex_init(&s_wdt_lock, "wdt_lock", RT_IPC_FLAG_PRIO);

    /* 1. Find & Configure Hardware Watchdog */
    s_hw_wdt_dev = rt_device_find(WDT_DEV_NAME);
    if (s_hw_wdt_dev != RT_NULL)
    {
        rt_err_t ret = rt_device_init(s_hw_wdt_dev);
        if (ret == RT_EOK)
        {
            rt_uint32_t timeout = TASK_WDT_HW_TIMEOUT_DEF;
            rt_device_control(s_hw_wdt_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &timeout);

            rt_uint32_t real_timeout = 0;
            rt_device_control(s_hw_wdt_dev, RT_DEVICE_CTRL_WDT_GET_TIMEOUT, &real_timeout);

            ret = rt_device_control(s_hw_wdt_dev, RT_DEVICE_CTRL_WDT_START, RT_NULL);
            if (ret == RT_EOK)
            {
                LOG_I("Hardware Watchdog armed successfully: Device='%s', Timeout=%d s", WDT_DEV_NAME, real_timeout);
            }
            else
            {
                LOG_E("Failed to start Hardware Watchdog device!");
            }
        }
        else
        {
            LOG_E("Failed to init Hardware Watchdog device!");
        }
    }
    else
    {
        LOG_W("Hardware Watchdog device '%s' not found! Task Watchdog running in software-only mode.", WDT_DEV_NAME);
    }

    /* 2. Register standard core system tasks with generous startup timeouts */
    task_wdt_register("rs232",    30);  /* PC RS232 thread */
    task_wdt_register("ble",      30);  /* BLE 5.0 UART thread */
    task_wdt_register("gnss",     30);  /* Beidou GNSS thread */
    task_wdt_register("ntn",      60);  /* MS150 satellite thread */
    /* V3 replay runs in raw_tx, which registers its own watchdog heartbeat. */

    /* 3. Start watchdog daemon thread */
    s_wdt_thread = rt_thread_create("wdt_mgr",
                                    task_wdt_daemon_entry,
                                    RT_NULL,
                                    1536,
                                    6,      /* High priority 6 */
                                    10);
    if (s_wdt_thread != RT_NULL)
    {
        rt_thread_startup(s_wdt_thread);
    }
    else
    {
        LOG_E("Failed to create Watchdog supervisor thread!");
    }

    s_wdt_init_ok = RT_TRUE;
    LOG_I("Multi-Task Heartbeat Watchdog Framework initialized.");
    return RT_EOK;
}
/* Initialize the registry before any service or business task registers itself. */
INIT_COMPONENT_EXPORT(task_wdt_init);

/* MSH Diagnostics */
static void wdt_status(int argc, char **argv)
{
    rt_kprintf("\n============= Multi-Task Watchdog Status =============\n");

    if (s_hw_wdt_dev != RT_NULL)
    {
        rt_uint32_t timeleft = 0;
        rt_device_control(s_hw_wdt_dev, RT_DEVICE_CTRL_WDT_GET_TIMELEFT, &timeleft);
        rt_kprintf(" Hardware Watchdog : ARMED (%s, Timeleft=%d s)\n", WDT_DEV_NAME, timeleft);
    }
    else
    {
        rt_kprintf(" Hardware Watchdog : NOT AVAILABLE\n");
    }

    rt_kprintf(" Global Monitoring : %s\n\n", s_global_enabled ? "ENABLED" : "PAUSED (DEBUG)");

    rt_kprintf(" ---------------------------------------------------------\n");
    rt_kprintf(" | ID | %-12s | Timeout(s) | Elapsed(s) | Status | Active |\n", "Task Name");
    rt_kprintf(" ---------------------------------------------------------\n");

    rt_tick_t now = rt_tick_get();
    for (int i = 0; i < TASK_WDT_MAX_TASKS; i++)
    {
        if (s_tasks[i].is_used)
        {
            rt_uint32_t elapsed_s = (now - s_tasks[i].last_heartbeat_tick) / RT_TICK_PER_SECOND;
            rt_kprintf(" | %2d | %-12s | %10d | %10d | %-6s | %-6s |\n",
                       i,
                       s_tasks[i].name,
                       s_tasks[i].timeout_sec,
                       elapsed_s,
                       s_tasks[i].is_hung ? "HUNG" : "OK",
                       s_tasks[i].is_enabled ? "YES" : "NO");
        }
    }
    rt_kprintf(" =========================================================\n\n");
}
MSH_CMD_EXPORT(wdt_status, Display task heartbeat watchdog status);

static void wdt_kick(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: wdt_kick <task_name>\n");
        return;
    }

    task_wdt_heartbeat_by_name(argv[1]);
    rt_kprintf("Heartbeat sent to task '%s'.\n", argv[1]);
}
MSH_CMD_EXPORT(wdt_kick, Send heartbeat to task);

static void wdt_test_hang(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: wdt_test_hang <task_name>\n");
        rt_kprintf("Simulates a task deadlock by setting its timeout to 1 second and disabling heartbeats.\n");
        return;
    }

    const char *name = argv[1];
    for (int i = 0; i < TASK_WDT_MAX_TASKS; i++)
    {
        if (s_tasks[i].is_used && rt_strcmp(s_tasks[i].name, name) == 0)
        {
            s_tasks[i].timeout_sec = 2; /* 2s timeout */
            s_tasks[i].last_heartbeat_tick = rt_tick_get() - (10 * RT_TICK_PER_SECOND); /* Make it immediately expired */
            rt_kprintf("Simulated deadlock on task '%s'! Watchdog supervisor should trigger and reboot in ~10s...\n", name);
            return;
        }
    }

    rt_kprintf("Task '%s' not found in registry!\n", name);
}
MSH_CMD_EXPORT(wdt_test_hang, Simulate task deadlock to verify watchdog reset);

static void wdt_pause(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: wdt_pause <1|0>\n");
        return;
    }

    int pause = argv[1][0] - '0';
    s_global_enabled = (pause == 0) ? RT_TRUE : RT_FALSE;
    rt_kprintf("Watchdog monitoring is now %s\n", s_global_enabled ? "RESUMED" : "PAUSED (DEBUG)");
}
MSH_CMD_EXPORT(wdt_pause, Pause or resume watchdog feeding);

/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * The historical file name is retained in the MDK project, but V3 turns this
 * module into the single PC-port router. Legacy automatic telemetry and
 * top-level Modbus modes are intentionally disabled by the V3 requirements.
 */

#include "app_telemetry.h"
#include "param_manager.h"
#include "ntn_ms150.h"
#include "pc_rs232.h"
#include "raw_uplink.h"
#include "mcu_local_cmd.h"
#include "pc_maint_core.h"
#include <string.h>
#include <stdlib.h>

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "app.router"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static rt_bool_t s_router_initialized;
static pc_maint_ctx_t s_pc_maint;
static struct rt_mutex s_maint_lock;
static rt_thread_t s_maint_thread;

static rt_uint32_t maint_now_ms(void)
{
    return (rt_uint32_t)(((rt_uint64_t)rt_tick_get() * 1000U) / RT_TICK_PER_SECOND);
}

static rt_bool_t maint_enabled(void)
{
    param_config_t cfg;
    param_get_snapshot(&cfg);
    return cfg.work_mode == PARAM_MODE_RAW_UPLINK && cfg.pc_cfg_enabled;
}

static void maint_raw(void *user, const uint8_t *data, size_t len)
{
    RT_UNUSED(user);
    (void)raw_uplink_input(data, (rt_size_t)len);
}

static void maint_local_command(void *user, const uint8_t *data, size_t len)
{
    char response[2048];
    char final_response[160];
    mcu_cmd_action_t action;
    rt_err_t ret;
    RT_UNUSED(user);
    if (!mcu_local_cmd_execute(data, (rt_size_t)len, response,
                               sizeof(response), &action))
    {
        pc_rs232_send_str("\r\n+MCUERROR:18\r\nERROR\r\n");
        return;
    }
    pc_rs232_send_str(response);
    if (action.type != MCU_CMD_ACTION_APPLY_RS232_BAUD) return;
    /* PENDING is sent at old baud; TX has 500 ms to drain before switching. */
    rt_thread_mdelay(500U);
    ret = pc_rs232_set_baudrate(action.value);
    if (ret == RT_EOK)
        rt_snprintf(final_response, sizeof(final_response),
                    "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"baud\":%u}\r\nOK\r\n",
                    action.value);
    else
    {
        (void)param_set_rs232_baud(action.previous_value, RT_TRUE);
        rt_snprintf(final_response, sizeof(final_response),
                    "\r\n+MCUERROR:3\r\nERROR\r\n");
    }
    pc_rs232_send_str(final_response);
}

static const pc_maint_callbacks_t s_maint_callbacks = {
    maint_raw, maint_local_command, RT_NULL
};

static void maint_watch_thread(void *parameter)
{
    RT_UNUSED(parameter);
    for (;;)
    {
        rt_thread_mdelay(50U);
        if (rt_mutex_take(&s_maint_lock, RT_WAITING_FOREVER) == RT_EOK)
        {
            pc_maint_poll(&s_pc_maint, maint_now_ms(), maint_enabled(),
                          &s_maint_callbacks);
            rt_mutex_release(&s_maint_lock);
        }
    }
}

static void terminal_rs232_rx(const rt_uint8_t *data, rt_size_t len)
{
    rt_uint8_t mode;
    if (data == RT_NULL || len == 0U)
    {
        return;
    }
    mode = param_get_work_mode();
    if (mode == PARAM_MODE_AT_PASSTHROUGH)
    {
        ntn_bridge_process_pc_data(data, len);
    }
    else if (mode == PARAM_MODE_RAW_UPLINK)
    {
        rt_mutex_take(&s_maint_lock, RT_WAITING_FOREVER);
        pc_maint_input(&s_pc_maint, data, len, maint_now_ms(),
                       maint_enabled(), &s_maint_callbacks);
        rt_mutex_release(&s_maint_lock);
    }
}

int app_telemetry_init(void)
{
    if (s_router_initialized)
    {
        return RT_EOK;
    }
    pc_maint_init(&s_pc_maint);
    rt_mutex_init(&s_maint_lock, "mpc_esc", RT_IPC_FLAG_PRIO);
    s_maint_thread = rt_thread_create("pcmaint", maint_watch_thread, RT_NULL,
                                      2048U, 18U, 10U);
    if (s_maint_thread == RT_NULL) return -RT_ERROR;
    rt_thread_startup(s_maint_thread);
    pc_rs232_set_rx_handler(terminal_rs232_rx);
    ntn_bridge_set(param_get_work_mode() == PARAM_MODE_AT_PASSTHROUGH);
    s_router_initialized = RT_TRUE;
    LOG_I("V3 PC router: mode0 AT, mode1 RAW with guarded +++ maintenance");
    return RT_EOK;
}
INIT_APP_EXPORT(app_telemetry_init);

rt_bool_t app_telemetry_pc_config_active(void)
{
    return (s_router_initialized && pc_maint_active(&s_pc_maint)) ?
           RT_TRUE : RT_FALSE;
}

void app_telemetry_set_mode(rt_uint8_t mode)
{
    if (param_set_work_mode(mode, RT_TRUE) == RT_EOK)
    {
        ntn_bridge_set(mode == PARAM_MODE_AT_PASSTHROUGH);
        if (mode == PARAM_MODE_RAW_UPLINK)
        {
            ntn_service_request_reconnect();
        }
    }
}

static void terminal_mode(int argc, char **argv)
{
    int mode;
    if (argc != 2)
    {
        rt_kprintf("Usage: terminal_mode <0:AT|1:RAW>\n");
        return;
    }
    mode = atoi(argv[1]);
    if (mode < 0 || mode > 1)
    {
        rt_kprintf("Invalid mode\n");
        return;
    }
    app_telemetry_set_mode((rt_uint8_t)mode);
    rt_kprintf("mode=%u\n", param_get_work_mode());
}
MSH_CMD_EXPORT(terminal_mode, set V3 work mode 0 AT or 1 RAW);

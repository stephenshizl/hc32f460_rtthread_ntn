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
#include <stdlib.h>

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "app.router"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static rt_bool_t s_router_initialized;

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
        (void)raw_uplink_input(data, len);
    }
}

int app_telemetry_init(void)
{
    if (s_router_initialized)
    {
        return RT_EOK;
    }
    pc_rs232_set_rx_handler(terminal_rs232_rx);
    ntn_bridge_set(param_get_work_mode() == PARAM_MODE_AT_PASSTHROUGH);
    s_router_initialized = RT_TRUE;
    LOG_I("V3 PC router ready: mode0 AT passthrough, mode1 RAW uplink");
    return RT_EOK;
}
INIT_APP_EXPORT(app_telemetry_init);

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

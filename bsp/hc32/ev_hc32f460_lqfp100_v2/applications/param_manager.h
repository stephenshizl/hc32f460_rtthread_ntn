/* SPDX-License-Identifier: Apache-2.0 */

#ifndef __PARAM_MANAGER_H__
#define __PARAM_MANAGER_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PARAM_SCHEMA_VERSION          3U
#define PARAM_MAGIC                   0x43464733UL /* "CFG3" */

#define PARAM_MODE_AT_PASSTHROUGH     0U
#define PARAM_MODE_RAW_UPLINK         1U

/* Compatibility names used by pre-V3 modules. They can no longer be saved. */
#define PARAM_MODE_RAW_PASS           PARAM_MODE_AT_PASSTHROUGH
#define PARAM_MODE_NTN_BRIDGE         PARAM_MODE_AT_PASSTHROUGH
#define PARAM_MODE_AUTO_TELEMETRY     0xFDU
#define PARAM_MODE_MODBUS             0xFEU
#define PARAM_MODE_CUSTOM_CAN         0xFFU

#define PARAM_TRANSFER_RAW            0U
#define PARAM_TRANSFER_MODBUS         1U
#define PARAM_RX_QUERY                0U
#define PARAM_RX_URC_PUSH             1U

#define PARAM_HOST_MAX_LEN            253U
#define PARAM_APN_MAX_LEN             100U
#define PARAM_SN_MAX_LEN              32U
#define PARAM_BLE_NAME_MAX_LEN        31U
#define PARAM_DNS_ADDR_MAX_LEN        15U
#define PARAM_PARTITION_NAME          "kvdb"

#pragma pack(1)
typedef struct
{
    rt_uint32_t magic;
    rt_uint16_t schema_version;
    rt_uint16_t struct_size;
    rt_uint32_t revision;
    rt_uint32_t save_tick;

    rt_uint8_t  work_mode;
    rt_uint8_t  transfer_mode;
    rt_uint8_t  rx_mode;
    rt_uint8_t  dns_mode;
    rt_uint8_t  pc_cfg_enabled;
    rt_uint8_t  ntn_location_enabled;
    rt_uint8_t  asset_report_enabled;
    rt_uint8_t  asset_location_enabled;
    rt_uint8_t  location_report_enabled;
    rt_uint8_t  reserved_flags[3];

    rt_uint16_t remote_port;
    rt_uint16_t local_port;
    rt_uint16_t raw_block_len;
    rt_uint16_t raw_gap_ms;
    rt_uint16_t raw_wait_ms;
    rt_uint16_t reserved16;

    rt_uint32_t rs232_baud;
    rt_uint32_t report_interval_s;
    rt_uint32_t location_interval_s;

    char host[PARAM_HOST_MAX_LEN + 1U];
    char dns_primary[PARAM_DNS_ADDR_MAX_LEN + 1U];
    char dns_secondary[PARAM_DNS_ADDR_MAX_LEN + 1U];
    char ntn_apn[PARAM_APN_MAX_LEN + 1U];
    char dev_sn[PARAM_SN_MAX_LEN + 1U];
    char ble_name[PARAM_BLE_NAME_MAX_LEN + 1U];
    char dev_hw_ver[16];
    char dev_sw_ver[16];
    rt_uint64_t next_report_id;
    rt_uint32_t crc32;
} param_config_t;
#pragma pack()

int param_manager_init(void);
const param_config_t *param_get_active(void);
void param_get_snapshot(param_config_t *out_config);
rt_bool_t param_has_persisted_config(void);

rt_err_t param_save(void);
rt_err_t param_reset_factory(void);

rt_uint8_t param_get_work_mode(void);
rt_err_t param_set_work_mode(rt_uint8_t mode, rt_bool_t save_now);
rt_uint8_t param_get_transfer_mode(void);
rt_err_t param_set_transfer_mode(rt_uint8_t mode, rt_bool_t save_now);

rt_uint32_t param_get_report_interval(void);
rt_err_t param_set_report_interval(rt_uint32_t seconds, rt_bool_t save_now);
rt_uint32_t param_get_rs232_baud(void);
rt_err_t param_set_rs232_baud(rt_uint32_t baud, rt_bool_t save_now);

const char *param_get_ntn_server(rt_uint16_t *out_port);
rt_err_t param_set_ntn_server(const char *host, rt_uint16_t port, rt_bool_t save_now);
rt_err_t param_set_host(const char *host, rt_bool_t save_now);
rt_err_t param_set_ports(rt_uint16_t remote_port, rt_uint16_t local_port, rt_bool_t save_now);
rt_bool_t param_endpoint_is_configured(void);
rt_uint32_t param_get_endpoint_identity(void);
rt_err_t param_set_apn(const char *apn, rt_bool_t save_now);
rt_err_t param_set_dns(rt_uint8_t mode, const char *primary,
                       const char *secondary, rt_bool_t save_now);
rt_err_t param_set_rx_mode(rt_uint8_t mode, rt_bool_t save_now);
rt_err_t param_set_pc_cfg_enabled(rt_bool_t enabled, rt_bool_t save_now);
rt_err_t param_set_ntn_location_enabled(rt_bool_t enabled, rt_bool_t save_now);
rt_err_t param_set_asset_report(rt_bool_t enabled, rt_uint32_t period_s,
                                rt_bool_t save_now);
rt_err_t param_set_location_report(rt_bool_t enabled, rt_uint32_t period_s,
                                   rt_bool_t save_now);
rt_err_t param_set_asset_location(rt_bool_t enabled, rt_bool_t save_now);
rt_err_t param_reserve_report_ids(rt_uint32_t count, rt_uint64_t *out_first_id);
rt_err_t param_set_raw_pack(rt_uint16_t gap_ms, rt_uint16_t block_len,
                            rt_uint16_t wait_ms, rt_bool_t save_now);

const char *param_get_ble_name(void);
rt_err_t param_set_ble_name(const char *name, rt_bool_t save_now);
const char *param_get_dev_sn(void);
rt_err_t param_set_dev_sn(const char *sn, rt_bool_t save_now);

rt_bool_t param_host_is_valid(const char *host);

/* Reset operational configuration while preserving factory identity fields. */
rt_err_t param_reset_operational(void);

#ifdef __cplusplus
}
#endif

#endif /* __PARAM_MANAGER_H__ */

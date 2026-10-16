/*
 * Copyright (c) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-14     Stephen      Stage 5.2: Device Parameter NVRAM Management Framework
 */

#ifndef __PARAM_MANAGER_H__
#define __PARAM_MANAGER_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Operational modes */
#define PARAM_MODE_RAW_PASS         0   /* 0: Transparent RS232 Raw Bridge (IR-9680 compatible) */
#define PARAM_MODE_NTN_BRIDGE       0   /* Alias: Default NTN Transparent Modem Bridge (PC treats MCU as NTN) */
#define PARAM_MODE_AUTO_TELEMETRY   1   /* 1: Auto GNSS / Sensor Telemetry Report */
#define PARAM_MODE_MODBUS           2   /* 2: Modbus RTU Master Polling Mode */
#define PARAM_MODE_CUSTOM_CAN       3   /* 3: Reserved: Custom CAN / Sensor Protocol Mode */

#define PARAM_STR_MAX_LEN           32
#define PARAM_SN_MAX_LEN            24
#define PARAM_PARTITION_NAME        "kvdb"
#define PARAM_MAGIC                 0x5041524D  /* ASCII 'PARM' */

#pragma pack(1)
/**
 * @brief Terminal Core Configuration Structure (Persistent NVRAM)
 */
typedef struct
{
    rt_uint32_t magic;                          /* 0x5041524D ("PARM") */
    rt_uint32_t version;                        /* Configuration revision counter */
    rt_uint32_t save_tick;                      /* System tick when saved */

    /* Terminal Operational Settings */
    rt_uint8_t  work_mode;                      /* PARAM_MODE_RAW_PASS, PARAM_MODE_AUTO_TELEMETRY, PARAM_MODE_MODBUS */
    rt_uint8_t  reserved1;                      /* Alignment padding */
    rt_uint16_t rs232_idle_ms;                  /* RS232 idle frame timeout (ms, default 20) */
    rt_uint32_t rs232_baud;                     /* RS232 baudrate (default 19200) */
    rt_uint32_t report_interval_s;              /* Telemetry auto report period in seconds (default 60) */

    /* Connectivity Settings */
    char        ble_name[PARAM_STR_MAX_LEN];    /* BLE broadcast name (default "NTN-Terminal") */
    char        ntn_apn[PARAM_STR_MAX_LEN];     /* Satellite APN (default "iridium.iot") */
    char        ntn_srv_ip[PARAM_STR_MAX_LEN];  /* Satellite Remote UDP/TCP Server IP/Host */
    rt_uint16_t ntn_srv_port;                   /* Satellite Remote Server Port (default 8888) */
    rt_uint8_t  ntn_socket_id;                  /* Socket ID (1-4, default 1) */
    rt_uint8_t  log_output_level;               /* Log level (0:off, 1:err, 2:info, 3:debug) */

    /* Device Identity */
    char        dev_sn[PARAM_SN_MAX_LEN];       /* Terminal Serial Number */
    char        dev_hw_ver[16];                 /* Hardware version */
    char        dev_sw_ver[16];                 /* Firmware version */

    rt_uint32_t crc32;                          /* CRC-32 over all preceding fields */
} param_config_t;
#pragma pack()

/**
 * @brief Initialize Parameter Manager, mount FAL "kvdb" partition, and load NVRAM configuration
 * @return RT_EOK on success, negative error code on failure
 */
int param_manager_init(void);

/**
 * @brief Get read-only pointer to active system configuration
 * @return Const pointer to configuration struct
 */
const param_config_t *param_get_active(void);

/**
 * @brief Save current active configuration into Flash (Ping-Pong dual sector fail-safe write)
 * @return RT_EOK on success, negative error code on failure
 */
rt_err_t param_save(void);

/**
 * @brief Reset all parameters to factory defaults and commit to Flash
 * @return RT_EOK on success, negative error code on failure
 */
rt_err_t param_reset_factory(void);

/**
 * @brief Get operational mode
 * @return PARAM_MODE_RAW_PASS, PARAM_MODE_AUTO_TELEMETRY, or PARAM_MODE_MODBUS
 */
rt_uint8_t param_get_work_mode(void);

/**
 * @brief Set operational mode
 * @param mode PARAM_MODE_RAW_PASS, PARAM_MODE_AUTO_TELEMETRY, or PARAM_MODE_MODBUS
 * @param save_now RT_TRUE to commit to Flash immediately
 * @return RT_EOK on success
 */
rt_err_t param_set_work_mode(rt_uint8_t mode, rt_bool_t save_now);

/**
 * @brief Get telemetry auto report interval in seconds
 */
rt_uint32_t param_get_report_interval(void);

/**
 * @brief Set telemetry auto report interval in seconds
 * @param seconds Interval in seconds (10 .. 86400)
 * @param save_now RT_TRUE to commit to Flash immediately
 */
rt_err_t param_set_report_interval(rt_uint32_t seconds, rt_bool_t save_now);

/**
 * @brief Get RS232 baudrate
 */
rt_uint32_t param_get_rs232_baud(void);

/**
 * @brief Set RS232 baudrate
 */
rt_err_t param_set_rs232_baud(rt_uint32_t baud, rt_bool_t save_now);

/**
 * @brief Get satellite remote server configuration
 * @param out_port Pointer to store server port
 * @return Pointer to server IP/host string
 */
const char *param_get_ntn_server(rt_uint16_t *out_port);

/**
 * @brief Set satellite remote server configuration
 * @param ip_or_host Server IP or domain
 * @param port Server port
 * @param save_now RT_TRUE to commit to Flash immediately
 */
rt_err_t param_set_ntn_server(const char *ip_or_host, rt_uint16_t port, rt_bool_t save_now);

/**
 * @brief Get BLE broadcast name
 */
const char *param_get_ble_name(void);

/**
 * @brief Set BLE broadcast name
 */
rt_err_t param_set_ble_name(const char *name, rt_bool_t save_now);

/**
 * @brief Get device serial number
 */
const char *param_get_dev_sn(void);

/**
 * @brief Set device serial number
 */
rt_err_t param_set_dev_sn(const char *sn, rt_bool_t save_now);

#ifdef __cplusplus
}
#endif

#endif /* __PARAM_MANAGER_H__ */

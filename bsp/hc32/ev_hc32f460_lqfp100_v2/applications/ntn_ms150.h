/*
 * Copyright (c) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-14     Stephen      First version: MS150 Iridium NTN satellite modem driver
 */

#ifndef __NTN_MS150_H__
#define __NTN_MS150_H__

#include <rtthread.h>
#include <rtdevice.h>
#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NTN_UART_NAME           "uart3"         /* HC32F460 USART3: PB4(TX), PB5(RX) */
#define NTN_DEFAULT_BAUDRATE    BAUD_RATE_115200/* MS150 standard baudrate */
#define NTN_RESP_BUF_MAX_SZ     512             /* Response buffer size */
#define NTN_DEFAULT_TIMEOUT_MS  3000            /* Default AT timeout in ms */

/**
 * @brief NTN Network Registration Status (from AT+CEREG?)
 */
typedef enum
{
    NTN_REG_NOT_REGISTERED  = 0,    /* 0: Not registered, not searching */
    NTN_REG_HOME            = 1,    /* 1: Registered, home network */
    NTN_REG_SEARCHING       = 2,    /* 2: Not registered, searching satellite network */
    NTN_REG_DENIED          = 3,    /* 3: Registration denied */
    NTN_REG_UNKNOWN         = 4,    /* 4: Unknown / No satellite coverage */
    NTN_REG_ROAMING         = 5     /* 5: Registered, roaming */
} ntn_reg_status_t;

/**
 * @brief Modem Hardware and Identity Information
 */
typedef struct
{
    char        model[32];          /* AT+CGMM */
    char        vendor[32];         /* AT+CGMI */
    char        fw_version[64];     /* AT+CGMR */
    char        imei[24];           /* AT+CGSN=1 */
    char        sn[32];             /* AT+CGSN=0 */
    char        iccid[24];          /* AT+ICCID */
    char        imsi[24];           /* AT+CIMI */
} ntn_modem_info_t;

/**
 * @brief Satellite RF and Signal Metrics
 */
typedef struct
{
    ntn_reg_status_t reg_status;    /* Network registration status */
    rt_uint8_t       rssi;          /* Signal RSSI (0-31, 99=not known) */
    rt_uint8_t       ber;           /* Bit error rate (0-7, 99=not known) */
    rt_int16_t       rsrp;          /* Reference Signal Received Power in dBm */
    rt_int16_t       rsrq;          /* Reference Signal Received Quality in dB */
    rt_int16_t       snr;           /* Signal to Noise Ratio in dB */
    rt_uint32_t      cell_id;       /* Satellite cell ID */
    rt_tick_t        last_check_tick;/* Tick of last network poll */
} ntn_signal_t;

/**
 * @brief Diagnostic traffic counters
 */
typedef struct
{
    rt_uint32_t tx_bytes;          /* Total bytes written to uart3 */
    rt_uint32_t rx_bytes;          /* Total bytes received from uart3 */
    rt_uint32_t at_cmds_sent;      /* Total AT commands sent */
    rt_uint32_t at_cmds_ok;        /* Commands returning OK */
    rt_uint32_t at_cmds_err;       /* Commands returning ERROR/timeout */
    rt_uint32_t bridge_pc_rx;      /* Bytes received from PC RS232 in bridge mode */
    rt_uint32_t bridge_ntn_to_pc;  /* Bytes forwarded from NTN to PC */
} ntn_stats_t;

/**
 * @brief Initialize MS150 NTN module driver and UART3 interface
 * @return RT_EOK on success, error code on failure
 */
int ntn_ms150_init(void);

/**
 * @brief Power control for MS150 modem and external PA
 * @param on RT_TRUE to power up modem + PA, RT_FALSE to power down
 */
void ntn_power_set(rt_bool_t on);

/**
 * @brief Hardware reset pulse to MS150 (PA8 high active pulse)
 * @param pulse_ms Pulse width in milliseconds (default 200ms)
 */
void ntn_reset(rt_uint32_t pulse_ms);

/**
 * @brief Execute an AT command synchronously and wait for response
 * @param cmd Null-terminated AT command string (e.g. "AT+CSQ")
 * @param resp_buf Buffer to store response, or RT_NULL if response content ignored
 * @param buf_sz Size of response buffer
 * @param timeout_ms Timeout in milliseconds
 * @return RT_EOK on OK, -RT_ETIMEOUT on timeout, -RT_ERROR on ERROR
 */
rt_err_t ntn_exec_cmd(const char *cmd, char *resp_buf, rt_size_t buf_sz, rt_uint32_t timeout_ms);

/**
 * @brief Send raw bytes directly to NTN module UART
 * @param data Data buffer
 * @param len Length in bytes
 * @return Bytes sent
 */
rt_size_t ntn_send_raw(const void *data, rt_size_t len);

/**
 * @brief Query modem identity (IMEI, IMSI, ICCID, Firmware version)
 * @param out_info Pointer to output identity structure
 * @return RT_EOK on success
 */
rt_err_t ntn_get_info(ntn_modem_info_t *out_info);

/**
 * @brief Query current satellite registration status and signal metrics
 * @param out_sig Pointer to output signal structure
 * @return RT_EOK on success
 */
rt_err_t ntn_get_signal(ntn_signal_t *out_sig);

/**
 * @brief Push latest GNSS coordinates into MS150 (AT+LOCATION=0.<lat>.<lon>.<alt>)
 * Automatically fetches coordinates from GNSS driver
 * @return RT_EOK on success
 */
rt_err_t ntn_sync_gnss_location(void);

/**
 * @brief Enable or disable PC RS232 (uart1) <-> MS150 (uart3) transparent debug bridge
 * When enabled, PC RS232 frames are directly forwarded to MS150, and MS150 replies
 * are forwarded back to PC RS232.
 * @param enable RT_TRUE to enable, RT_FALSE to disable
 */
void ntn_bridge_set(rt_bool_t enable);

/**
 * @brief Check if PC transparent bridge is currently active
 */
rt_bool_t ntn_bridge_is_enabled(void);

/**
 * @brief Process PC RS232 data in NTN bridge mode with line assembly and incomplete packet guard
 * @param data Pointer to incoming buffer
 * @param len  Length of buffer
 */
void ntn_bridge_process_pc_data(const rt_uint8_t *data, rt_size_t len);

/**
 * @brief Send UDP hex payload over satellite network via MS150
 * @param socket_id Socket ID (1-4)
 * @param hex_payload Hex-encoded ASCII string (e.g. "0031323334")
 * @return RT_EOK on success
 */
rt_err_t ntn_send_udp_hex(rt_uint8_t socket_id, const char *hex_payload);

/**
 * @brief Get diagnostic traffic statistics
 * @param out_stats Pointer to statistics structure
 */
void ntn_get_stats(ntn_stats_t *out_stats);

#ifdef __cplusplus
}
#endif

#endif /* __NTN_MS150_H__ */

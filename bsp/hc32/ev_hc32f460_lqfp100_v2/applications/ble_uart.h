/*
 * Copyright (c) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-13     Stephen      First version: BLE UART driver on uart4 (PB6/PB7)
 */

#ifndef __BLE_UART_H__
#define __BLE_UART_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_UART_NAME           "uart4"         /* HC32F460 USART4: PB6(TX), PB7(RX) */
#define BLE_DEFAULT_BAUDRATE    BAUD_RATE_115200/* Standard 115200 bps for BLE SoC */
#define BLE_RX_BUF_MAX_SZ       1024            /* Frame buffer capacity */
#define BLE_IDLE_TIMEOUT_MS     15              /* 15ms frame boundary timeout */

/* ==================== BLE Frame Protocol Constants ==================== */
#define BLE_FRAME_SOF1          0xAA
#define BLE_FRAME_SOF2          0x55

/* Command Types (MsgType) */
#define BLE_CMD_SYS_STATUS      0x01    /* System status, battery, Flash storage query */
#define BLE_CMD_GNSS_INFO       0x02    /* Real-time GNSS positioning and time query */
#define BLE_CMD_NTN_AT_PASS     0x03    /* NTN modem AT command pass-through */
#define BLE_CMD_LOG_STREAM      0x04    /* Wireless real-time log output stream control */
#define BLE_CMD_DEV_CONFIG      0x05    /* Device profile and mode configuration */
#define BLE_CMD_OTA_UPGRADE     0x06    /* In-field firmware OTA packet transport */
#define BLE_CMD_SECURITY_AUTH   0x07    /* Security authentication and pairing */

/**
 * @brief BLE Raw frame receive callback prototype
 * @param frame Pointer to the complete received frame data
 * @param size  Frame length in bytes
 */
typedef void (*ble_rx_handler_t)(const rt_uint8_t *frame, rt_size_t size);

/**
 * @brief Initialize BLE UART interface on uart4 (115200 8N1)
 * @return RT_EOK on success, error code on failure
 */
int ble_uart_init(void);

/**
 * @brief Register upper-layer frame callback handler
 * @param handler Callback function pointer
 */
void ble_uart_set_rx_handler(ble_rx_handler_t handler);

/**
 * @brief Send raw buffer to BLE SoC
 * @param buf Data buffer pointer
 * @param len Data length in bytes
 * @return Bytes actually sent
 */
rt_size_t ble_uart_send(const void *buf, rt_size_t len);

/**
 * @brief Send null-terminated string to BLE SoC
 * @param str String pointer
 * @return Bytes actually sent
 */
rt_size_t ble_uart_send_str(const char *str);

/**
 * @brief Pack and send standard protocol frame to Phone App over BLE
 * @param msg_type Command type code (BLE_CMD_*)
 * @param seq      Sequence number
 * @param payload  Data payload buffer
 * @param len      Payload length in bytes
 * @return Total bytes transmitted (including frame envelope and CRC)
 */
rt_size_t ble_send_frame(rt_uint8_t msg_type, rt_uint8_t seq, const rt_uint8_t *payload, rt_uint16_t len);

/**
 * @brief Check if wireless real-time log streaming to BLE is enabled
 */
rt_bool_t ble_log_stream_enabled(void);

/**
 * @brief Enable or disable live log output over BLE
 */
void ble_log_stream_set(rt_bool_t enable);

/**
 * @brief Enable or disable loopback echo test mode
 */
void ble_uart_set_loopback(rt_bool_t enable);

#ifdef __cplusplus
}
#endif

#endif /* __BLE_UART_H__ */

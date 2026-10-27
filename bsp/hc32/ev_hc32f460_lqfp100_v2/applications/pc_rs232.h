/*
 * Copyright (c) 2026, youyoutech.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-13     Stephen      First version: PC RS232 Driver with idle frame detection
 */

#ifndef __PC_RS232_H__
#define __PC_RS232_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RS232_UART_NAME         "uart1"         /* HC32F460 USART1: PA0(RX), PA1(TX) */
#define RS232_DEFAULT_BAUDRATE  BAUD_RATE_19200 /* Schema-3 safe default */
#define RS232_RX_FRAME_MAX_SZ   1024            /* Maximum frame buffer size */
#define RS232_IDLE_TIMEOUT_MS   20              /* Inter-character idle timeout for framing (ms) */

/**
 * @brief Frame receive callback function prototype
 * @param frame Pointer to the complete received frame data
 * @param size  Length of the received frame in bytes
 */
typedef void (*pc_rs232_rx_handler_t)(const rt_uint8_t *frame, rt_size_t size);

/**
 * @brief Initialize PC RS232 serial interface (19200 8N1, idle timeout framing)
 * @return RT_EOK on success, negative error code on failure
 */
int pc_rs232_init(void);

/**
 * @brief Register upper-layer byte-chunk receive callback
 * @param handler Callback function pointer
 */
void pc_rs232_set_rx_handler(pc_rs232_rx_handler_t handler);

/**
 * @brief Send raw buffer to PC over RS232
 * @param buf Data buffer pointer
 * @param len Data length in bytes
 * @return Bytes actually sent
 */
rt_size_t pc_rs232_send(const void *buf, rt_size_t len);

/**
 * @brief Send null-terminated string to PC over RS232
 * @param str String pointer
 * @return Bytes actually sent
 */
rt_size_t pc_rs232_send_str(const char *str);

/**
 * @brief Set RS232 baudrate dynamically
 * @param baud_rate Supported rates: 9600, 19200, 38400, 57600, 115200
 * @return RT_EOK on success, -RT_ERROR on failure
 */
rt_err_t pc_rs232_set_baudrate(rt_uint32_t baud_rate);

/**
 * @brief Get current RS232 baudrate
 */
rt_uint32_t pc_rs232_get_baudrate(void);
rt_bool_t pc_rs232_is_idle(rt_uint32_t quiet_ms);

/**
 * @brief Enable or disable loopback echo test mode
 * @param enable RT_TRUE to echo back received frames to PC
 */
void pc_rs232_set_loopback(rt_bool_t enable);

#ifdef __cplusplus
}
#endif

#endif /* __PC_RS232_H__ */

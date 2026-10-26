/* SPDX-License-Identifier: Apache-2.0 */
#ifndef __BLE_UART_H__
#define __BLE_UART_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_UART_NAME              "uart4"
#define BLE_DEFAULT_BAUDRATE       BAUD_RATE_115200
#define CTRL2_SYNC_1               0xAAU
#define CTRL2_SYNC_2               0x55U
#define CTRL2_VERSION              0x02U
#define CTRL2_MAX_PAYLOAD          2048U
#define CTRL2_RX_BUFFER_SIZE       4096U
#define CTRL2_TYPE_LOCAL_REQUEST   0x01U
#define CTRL2_TYPE_LOCAL_FINAL     0x02U
#define CTRL2_TYPE_NTN_REQUEST     0x03U
#define CTRL2_TYPE_NTN_FRAGMENT    0x04U
#define CTRL2_TYPE_NTN_ASYNC       0x05U
#define CTRL2_TYPE_EVENT           0x06U
#define CTRL2_TYPE_NTN_FINAL       0x07U
#define CTRL2_TYPE_LOCAL_PENDING   0x08U

rt_size_t ble_management_send(rt_uint8_t kind,rt_uint16_t id,const void *data,rt_uint16_t len);
int ble_uart_init(void);
rt_size_t ble_uart_send(const void *buf, rt_size_t len);
rt_size_t ble_ctrl2_send(rt_uint8_t type, rt_uint16_t request_id,
                         const void *payload, rt_uint16_t payload_len);

#ifdef __cplusplus
}
#endif
#endif

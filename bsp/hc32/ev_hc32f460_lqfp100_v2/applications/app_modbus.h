/*
 * Copyright (c) 2026, youyoutech.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-14     Stephen      Stage 5.4: Industrial Modbus RTU Master Auto-Polling Collector
 */

#ifndef __APP_MODBUS_H__
#define __APP_MODBUS_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MODBUS_MAX_CHANNELS         4       /* Maximum auto-polling channels */
#define MODBUS_MAX_REGS_PER_POLL    16      /* Max 16-bit registers per poll block */
#define MODBUS_DEFAULT_TIMEOUT_MS   1000    /* Slave response timeout in ms */

/* Modbus Function Codes */
#define MODBUS_FC_READ_HOLDING      0x03    /* Read Holding Registers (4x) */
#define MODBUS_FC_READ_INPUT        0x04    /* Read Input Registers (3x) */
#define MODBUS_FC_WRITE_SINGLE      0x06    /* Write Single Register */
#define MODBUS_FC_WRITE_MULTIPLE    0x10    /* Write Multiple Registers */

#pragma pack(1)
/**
 * @brief Compact NTAP Modbus Telemetry Packet for Satellite Transmission
 */
typedef struct
{
    rt_uint8_t  msg_type;       /* NTAP_MSG_TYPE_MODBUS_DATA (0x04) */
    rt_uint8_t  slave_addr;     /* Modbus Slave ID */
    rt_uint8_t  func_code;      /* 0x03 / 0x04 */
    rt_uint8_t  byte_count;     /* 2 * reg_count */
    rt_uint16_t start_reg;      /* Register start address */
    rt_uint16_t reg_count;      /* Register count */
    rt_uint32_t timestamp;      /* System tick when polled */
    rt_uint16_t reg_values[MODBUS_MAX_REGS_PER_POLL]; /* Polled 16-bit register values */
    rt_uint16_t payload_crc16;  /* Packet integrity CRC16 */
} ntap_modbus_pkt_t;
#pragma pack()

/**
 * @brief Modbus polling channel definition
 */
typedef struct
{
    rt_uint8_t  slave_addr;     /* Slave ID (1..247) */
    rt_uint8_t  func_code;      /* MODBUS_FC_READ_HOLDING or MODBUS_FC_READ_INPUT */
    rt_uint16_t start_reg;      /* Register address (e.g. 0x0000) */
    rt_uint16_t reg_count;      /* Number of registers to read (1..16) */
    rt_uint16_t poll_period_s;  /* Polling interval in seconds */
    rt_bool_t   is_enabled;     /* Channel enabled flag */

    /* Runtime Cache */
    rt_uint16_t last_regs[MODBUS_MAX_REGS_PER_POLL]; /* Last successfully read register values */
    rt_tick_t   last_poll_tick; /* Tick of last poll attempt */
    rt_bool_t   last_poll_ok;   /* Status of last poll */
} modbus_channel_t;

/**
 * @brief Diagnostic statistics for Modbus Master
 */
typedef struct
{
    rt_uint32_t polls_sent;         /* Total poll requests sent */
    rt_uint32_t polls_ok;           /* Successful poll responses received */
    rt_uint32_t polls_timeout;      /* Slave response timeouts */
    rt_uint32_t polls_crc_err;      /* Slave response CRC mismatches */
    rt_uint32_t polls_exception;    /* Modbus exception responses */
    rt_uint32_t packets_enqueued;   /* Total Modbus packets pushed to Store&Forward */
} modbus_stats_t;

/**
 * @brief Initialize Modbus RTU Master subsystem and start polling thread
 * @return RT_EOK on success, negative error code on failure
 */
int app_modbus_init(void);

/**
 * @brief Callback invoked when a frame is received from RS232 in MODBUS mode
 * @param data Received frame buffer
 * @param len Frame length
 */
void app_modbus_on_rx_frame(const rt_uint8_t *data, rt_size_t len);

/**
 * @brief Synchronous Modbus read holding/input registers
 * @param slave Slave address (1..247)
 * @param fc Function code (0x03 or 0x04)
 * @param start_reg Register address
 * @param reg_count Number of registers (1..16)
 * @param out_regs Buffer to receive 16-bit register values
 * @param timeout_ms Response timeout in milliseconds
 * @return RT_EOK on success, negative error code on failure
 */
rt_err_t app_modbus_read_registers(rt_uint8_t slave, rt_uint8_t fc, rt_uint16_t start_reg,
                                  rt_uint16_t reg_count, rt_uint16_t *out_regs, rt_uint32_t timeout_ms);

/**
 * @brief Synchronous Modbus write single register (FC 0x06)
 * @param slave Slave address (1..247)
 * @param reg Register address
 * @param value 16-bit value to write
 * @param timeout_ms Response timeout in milliseconds
 * @return RT_EOK on success, negative error code on failure
 */
rt_err_t app_modbus_write_single_register(rt_uint8_t slave, rt_uint16_t reg, rt_uint16_t value, rt_uint32_t timeout_ms);

/**
 * @brief Manually poll a specific channel and push result to Store & Forward
 * @param channel_idx Channel index (0 .. MODBUS_MAX_CHANNELS - 1)
 * @return RT_EOK on success
 */
rt_err_t app_modbus_poll_channel(rt_uint8_t channel_idx);

/**
 * @brief Get diagnostic statistics
 * @param out_stats Pointer to output statistics structure
 */
void app_modbus_get_stats(modbus_stats_t *out_stats);

/**
 * @brief Get copy of polling channel configuration
 * @param channel_idx Channel index
 * @param out_ch Pointer to output channel struct
 * @return RT_EOK on success
 */
rt_err_t app_modbus_get_channel(rt_uint8_t channel_idx, modbus_channel_t *out_ch);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MODBUS_H__ */

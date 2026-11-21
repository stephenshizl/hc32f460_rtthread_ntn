/* Copyright (c) 2026, youyoutech.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef NTN_OTA_BL1_ICG_H
#define NTN_OTA_BL1_ICG_H

/* Enable SWDT from reset: 4096 cycles / DIV64 @ 9-11 kHz = 23.83-29.13 s (typ 26.21 s),
 * 0-100% refresh window, reset on overflow, continue counting in sleep/stop so a stalled
 * trial cannot bypass rollback via WFI. WDT stays software-started for App task_wdt. */
#define NTN_OTA_SWDT_CONFIG (ICG_SWDT_RST_START | ICG_SWDT_EXP_TYPE_RST | \
    ICG_SWDT_CNT_PERIOD4096 | ICG_SWDT_CLK_DIV64 | ICG_SWDT_RANGE_0TO100PCT | \
    ICG_SWDT_LPM_CNT_CONT)

#define ICG_REG_CFG0_CONST (ICG_REG_WDT_CONFIG | NTN_OTA_SWDT_CONFIG | 0xE000E000UL)

/* Enable BOR at Level 3 (2.30-2.51 V SuperSpeed / 2.20-2.40 V HighSpeed); preserve
 * 16 MHz HRC oscillation, NMI disabled, and reserved bits all ones. */
#define ICG_REG_CFG1_CONST (ICG_REG_NMI_CONFIG | \
    (ICG_BOR_VOL_THRESHOLD_LVL3 | ICG_BOR_RST_ENABLE) | \
    ICG_REG_HRC_CONFIG | 0x03F8FEFEUL)

#endif

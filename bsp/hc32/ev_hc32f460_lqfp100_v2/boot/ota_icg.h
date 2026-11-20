/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NTN_OTA_BL1_ICG_H
#define NTN_OTA_BL1_ICG_H

/* Enable SWDT from reset with the DDL's longest period and full feed
 * window. Keep counting in sleep so a stalled trial cannot feed itself. */
#define ICG_REG_CFG0_CONST (ICG_REG_WDT_CONFIG | \
    (ICG_REG_SWDT_CONFIG & ~(ICG_ICG0_SWDTAUTS | ICG_ICG0_SWDTSLPOFF)) | 0xE000E000UL)
#endif

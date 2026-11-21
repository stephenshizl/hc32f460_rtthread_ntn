/* Copyright (c) 2026, youyoutech.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef NTN_OTA_LAYOUT_H
#define NTN_OTA_LAYOUT_H

#define NTN_OTA_LAYOUT_ID       1
#define NTN_OTA_BOOT_ABI        2
#define NTN_OTA_PRODUCT_ID      0x314E544E
#define NTN_OTA_HW_REV          1
#define NTN_OTA_BL1_BASE        0x00000000
#define NTN_OTA_BL1_SIZE        0x00004000
#define NTN_OTA_ICG_ADDR        0x00000400
#define NTN_OTA_ICG_SIZE        0x20
#define NTN_OTA_BL2_BASE        0x00004000
#define NTN_OTA_BL2_SIZE        0x0001C000
#define NTN_OTA_PRIMARY         0x00020000
#define NTN_OTA_SLOT_SIZE       0x0005E000
#define NTN_OTA_HEADER_SIZE     0x00000400
#define NTN_OTA_APP_VECTOR      (NTN_OTA_PRIMARY + NTN_OTA_HEADER_SIZE)
#define NTN_OTA_APP_MAX         0x0005B400
#define NTN_OTA_TLV_MAX         0x00000800
#define NTN_OTA_TRAILER_MAX     0x00002000
#define NTN_OTA_DESCRIPTOR_OFF  0x00000400
#define NTN_OTA_ONCHIP_SECTOR   0x2000
#define NTN_OTA_NOR_SECTOR      0x1000
#define NTN_OTA_NOR_SIZE        0x800000
#define NTN_OTA_SECONDARY       0x000000
#define NTN_OTA_FACTORY         0x080000
#define NTN_OTA_FACTORY_SIZE    0x080000
#define NTN_OTA_KVDB            0x100000
#define NTN_OTA_KVDB_SIZE       0x040000
#define NTN_OTA_QUEUE           0x140000
#define NTN_OTA_QUEUE_SIZE      0x0C0000
#define NTN_OTA_SCRATCH         0x200000
#define NTN_OTA_SCRATCH_SIZE    0x010000
#define NTN_OTA_LOG_A           0x210000
#define NTN_OTA_LOG_B           0x211000
#define NTN_OTA_NOR_RESERVED    0x212000
#define NTN_OTA_MAILBOX         0x1FFF8008
#define NTN_OTA_MAILBOX_SIZE    0x10
#define NTN_OTA_PACKAGE_MAX     (NTN_OTA_SLOT_SIZE - NTN_OTA_TRAILER_MAX)
/* MCUboot 2.4 scratch with default BOOT_MAX_ALIGN (8) and 16-byte magic. */
#define NTN_OTA_IMAGE_OK_OFF    (NTN_OTA_SLOT_SIZE - 24)

#endif

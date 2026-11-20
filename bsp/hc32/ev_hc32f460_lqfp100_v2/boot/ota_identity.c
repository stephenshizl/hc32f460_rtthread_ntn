/* Copyright (c) 2026, youyoutech.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include "ota_layout.h"
#include "ota_build_id.h"

struct ntn_product_descriptor
{
    uint8_t magic[4];
    uint16_t format;
    uint16_t size;
    uint32_t product_id;
    uint32_t layout_id;
    uint32_t hardware_min;
    uint32_t hardware_max;
    uint32_t min_boot_abi;
    uint8_t build_id[16];
    uint8_t reserved[20];
};

typedef char descriptor_must_be_64_bytes[sizeof(struct ntn_product_descriptor) == 64 ? 1 : -1];

__attribute__((used, section(".ntn_product")))
const struct ntn_product_descriptor ntn_product =
{
    {'N', 'T', 'H', 'C'}, 1, 64, NTN_OTA_PRODUCT_ID, NTN_OTA_LAYOUT_ID,
    NTN_OTA_HW_REV, NTN_OTA_HW_REV, NTN_OTA_BOOT_ABI, NTN_OTA_BUILD_ID, {0}
};

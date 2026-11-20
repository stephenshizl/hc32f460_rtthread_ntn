/* Copyright (c) 2026, youyoutech.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef NTN_OTA_MAILBOX_H
#define NTN_OTA_MAILBOX_H
#include <stdint.h>
#include "ota_layout.h"

#define NTN_OTA_ENTER_MAGIC 0x4E544E42UL

static inline void ntn_ota_request_boot(void)
{
    volatile uint32_t *mailbox = (volatile uint32_t *)NTN_OTA_MAILBOX;
    mailbox[1] = ~NTN_OTA_ENTER_MAGIC;
    mailbox[0] = NTN_OTA_ENTER_MAGIC;
}

static inline int ntn_ota_take_boot_request(void)
{
    volatile uint32_t *mailbox = (volatile uint32_t *)NTN_OTA_MAILBOX;
    int result = mailbox[0] == NTN_OTA_ENTER_MAGIC && mailbox[1] == ~NTN_OTA_ENTER_MAGIC;
    mailbox[0] = 0;
    mailbox[1] = 0;
    return result;
}
#endif

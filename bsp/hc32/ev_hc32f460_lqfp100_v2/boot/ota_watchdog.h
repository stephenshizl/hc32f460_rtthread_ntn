/* Copyright (c) 2026, youyoutech.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef NTN_OTA_WATCHDOG_H
#define NTN_OTA_WATCHDOG_H

#include "hc32_ll.h"

static inline void ntn_ota_feed_watchdog(void)
{
    if ((CM_ICG->ICG0 & ICG_ICG0_SWDTAUTS) == 0)
        SWDT_FeedDog();
}
#endif

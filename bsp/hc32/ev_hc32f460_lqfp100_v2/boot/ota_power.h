/* Copyright (c) 2026, youyoutech.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef NTN_OTA_POWER_H
#define NTN_OTA_POWER_H

#include <stdbool.h>

bool ntn_ota_power_init(void);
bool ntn_ota_voltage_ok(void);
bool ntn_ota_last_power_fault(void);
void ntn_ota_clear_power_fault(void);
void ntn_ota_efm_lock_all(void);

#endif

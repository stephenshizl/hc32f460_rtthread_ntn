/* Copyright (c) 2026, youyoutech.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef NTN_MCUBOOT_CONFIG_H
#define NTN_MCUBOOT_CONFIG_H
#define MCUBOOT_SIGN_EC256
#define MCUBOOT_USE_TINYCRYPT
#define MCUBOOT_VALIDATE_PRIMARY_SLOT
#define MCUBOOT_SWAP_USING_SCRATCH 1
#define MCUBOOT_IMAGE_NUMBER 1
#define MCUBOOT_MAX_IMG_SECTORS 128
#define MCUBOOT_USE_FLASH_AREA_GET_SECTORS
#define MCUBOOT_IMAGE_ACCESS_HOOKS
#define MCUBOOT_HAVE_LOGGING 1
void ntn_ota_mcuboot_watchdog_feed(void);
#define MCUBOOT_WATCHDOG_FEED() ntn_ota_mcuboot_watchdog_feed()
#define MCUBOOT_CPU_IDLE() ((void)0)
#endif

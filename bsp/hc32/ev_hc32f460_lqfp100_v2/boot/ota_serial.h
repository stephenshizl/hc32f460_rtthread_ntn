/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NTN_OTA_SERIAL_H
#define NTN_OTA_SERIAL_H
#include <stdbool.h>
#include <stdint.h>
struct flash_area;
bool ntn_ota_check_image(const struct flash_area *area, uint32_t received);
int ntn_ota_serial(unsigned listen_ms, const char *status);
void ntn_ota_report_boot_error(const char *error);
#endif

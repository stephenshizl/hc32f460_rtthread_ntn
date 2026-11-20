/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NTN_OTA_APP_H
#define NTN_OTA_APP_H

#include <stdbool.h>

bool ntn_ota_is_confirmed(void);
bool ntn_ota_confirm(void);
void ntn_ota_get_build_id(char out[33]);
#endif

/* SPDX-License-Identifier: Apache-2.0 */

#ifndef __APP_TELEMETRY_H__
#define __APP_TELEMETRY_H__

#include <rtthread.h>

/* Retained only for the P2 Modbus adapter build; V3 P0 does not transmit it. */
#define NTAP_MSG_TYPE_MODBUS_DATA  0x04U

int app_telemetry_init(void);
void app_telemetry_set_mode(rt_uint8_t mode);

#endif /* __APP_TELEMETRY_H__ */

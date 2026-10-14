/*
 * Copyright (C) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-13     Antigravity  First version: PC <-> MCU <-> NTN transparent bridge
 * 2026-10-14     Stephen      Redirect to enhanced MS150 NTN driver (ntn_ms150.h)
 */

#ifndef __NTN_BRIDGE_H__
#define __NTN_BRIDGE_H__

#include "ntn_ms150.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Backward compatibility aliases */
static inline void ntn_hw_init(void)          { ntn_ms150_init(); }
static inline void ntn_power_control(rt_bool_t on) { ntn_power_set(on); }
static inline int  ntn_get_status_pin(void)   { return hw_ntn_status_get(); }
static inline int  ntn_bridge_init(void)      { return ntn_ms150_init(); }

#ifdef __cplusplus
}
#endif

#endif /* __NTN_BRIDGE_H__ */

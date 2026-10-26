/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BT_IDENTITY_H
#define BT_IDENTITY_H
#include <rtthread.h>
/* Called only by the idle existing NTN worker, using its exclusive command buffer. */
void bt_identity_poll(char *scratch,rt_size_t capacity);
void bt_identity_link_reply(rt_uint16_t id,const rt_uint8_t *data,rt_uint16_t len);
rt_bool_t bt_identity_command(const char *command,char *out,rt_size_t cap);
void bt_identity_sn_changed(void);
#endif

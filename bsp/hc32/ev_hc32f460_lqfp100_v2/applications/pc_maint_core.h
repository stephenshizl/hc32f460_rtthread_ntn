/* SPDX-License-Identifier: Apache-2.0 */
#ifndef PC_MAINT_CORE_H
#define PC_MAINT_CORE_H

#include <stddef.h>
#include <stdint.h>

/* Hardware-independent V3 guarded escape parser. Times are monotonic milliseconds. */
#define PC_MAINT_CANDIDATE_MAX 256U
#define PC_MAINT_COMMAND_MAX 384U
#define PC_MAINT_PRE_GUARD_MS 1000U
#define PC_MAINT_POST_GUARD_MS 1000U
#define PC_MAINT_PLUS_GAP_MS 100U
#define PC_MAINT_CONFIRM_MS 3000U
#define PC_MAINT_IDLE_MS 60000U

typedef enum {
    PC_MAINT_RAW = 0,
    PC_MAINT_PLUS1,
    PC_MAINT_PLUS2,
    PC_MAINT_POST,
    PC_MAINT_CONFIRM,
    PC_MAINT_SESSION,
    PC_MAINT_DRAIN
} pc_maint_state_t;

typedef void (*pc_maint_raw_cb)(void *user, const uint8_t *bytes, size_t len);
typedef void (*pc_maint_cmd_cb)(void *user, const uint8_t *line, size_t len);

typedef struct {
    pc_maint_raw_cb raw;
    pc_maint_cmd_cb command;
    void *user;
} pc_maint_callbacks_t;

typedef struct {
    pc_maint_state_t state;
    uint32_t last_byte_ms;
    uint32_t stage_ms;
    uint32_t session_activity_ms;
    uint8_t candidate[PC_MAINT_COMMAND_MAX];
    size_t length;
    uint8_t ever_received;
    uint8_t skip_next_lf;
} pc_maint_ctx_t;

void pc_maint_init(pc_maint_ctx_t *ctx);
void pc_maint_poll(pc_maint_ctx_t *ctx, uint32_t now_ms, int enabled,
                   const pc_maint_callbacks_t *cb);
void pc_maint_input(pc_maint_ctx_t *ctx, const uint8_t *bytes, size_t len,
                    uint32_t now_ms, int enabled, const pc_maint_callbacks_t *cb);
void pc_maint_reset(pc_maint_ctx_t *ctx, const pc_maint_callbacks_t *cb,
                    int replay_candidate);
int pc_maint_active(const pc_maint_ctx_t *ctx);

#endif

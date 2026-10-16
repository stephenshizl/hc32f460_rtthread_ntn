/* SPDX-License-Identifier: Apache-2.0 */
#include "pc_maint_core.h"
#include <string.h>

static void replay(pc_maint_ctx_t *ctx, const pc_maint_callbacks_t *cb)
{
    if (ctx->length != 0U && cb != NULL && cb->raw != NULL)
        cb->raw(cb->user, ctx->candidate, ctx->length);
    ctx->length = 0U;
    ctx->state = PC_MAINT_RAW;
}

void pc_maint_init(pc_maint_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
}

void pc_maint_reset(pc_maint_ctx_t *ctx, const pc_maint_callbacks_t *cb,
                    int replay_candidate)
{
    if (ctx->state != PC_MAINT_SESSION &&
        ctx->state != PC_MAINT_DRAIN && replay_candidate)
        replay(ctx, cb);
    ctx->state = PC_MAINT_RAW;
    ctx->length = 0U;
    ctx->skip_next_lf = 0U;
}

int pc_maint_active(const pc_maint_ctx_t *ctx)
{
    return ctx->state == PC_MAINT_SESSION || ctx->state == PC_MAINT_DRAIN;
}

void pc_maint_poll(pc_maint_ctx_t *ctx, uint32_t now_ms, int enabled,
                   const pc_maint_callbacks_t *cb)
{
    if (!enabled)
    {
        pc_maint_reset(ctx, cb, 1);
        return;
    }
    if ((ctx->state == PC_MAINT_PLUS1 || ctx->state == PC_MAINT_PLUS2) &&
        (uint32_t)(now_ms - ctx->stage_ms) > PC_MAINT_PLUS_GAP_MS)
        replay(ctx, cb);
    else if (ctx->state == PC_MAINT_POST &&
             (uint32_t)(now_ms - ctx->stage_ms) >= PC_MAINT_POST_GUARD_MS)
    {
        ctx->state = PC_MAINT_CONFIRM;
        ctx->stage_ms += PC_MAINT_POST_GUARD_MS;
        if ((uint32_t)(now_ms - ctx->stage_ms) >= PC_MAINT_CONFIRM_MS)
            replay(ctx, cb);
    }
    else if (ctx->state == PC_MAINT_CONFIRM &&
             (uint32_t)(now_ms - ctx->stage_ms) >= PC_MAINT_CONFIRM_MS)
        replay(ctx, cb);
    else if ((ctx->state == PC_MAINT_SESSION ||
              ctx->state == PC_MAINT_DRAIN) &&
             (uint32_t)(now_ms - ctx->session_activity_ms) >= PC_MAINT_IDLE_MS)
    {
        /* Never replay a partial maintenance command as telemetry. */
        ctx->state = PC_MAINT_RAW;
        ctx->length = 0U;
    }
}

static void emit_raw(const pc_maint_callbacks_t *cb, const uint8_t *buf, size_t len)
{
    if (len != 0U && cb != NULL && cb->raw != NULL)
        cb->raw(cb->user, buf, len);
}

void pc_maint_input(pc_maint_ctx_t *ctx, const uint8_t *bytes, size_t len,
                    uint32_t now_ms, int enabled, const pc_maint_callbacks_t *cb)
{
    uint8_t plain[PC_MAINT_CANDIDATE_MAX];
    size_t plain_len = 0U;
    size_t i;
    if (ctx == NULL || bytes == NULL || cb == NULL) return;
    pc_maint_poll(ctx, now_ms, enabled, cb);
    for (i = 0U; i < len; i++)
    {
        uint8_t ch = bytes[i];
        int retry = 1;
        /* The idle frame callback groups bytes: same callback shares timestamp. */
        while (retry)
        {
            retry = 0;
            if (ctx->skip_next_lf)
            {
                ctx->skip_next_lf = 0U;
                if (ch == '\n') break;
            }
            if (!enabled)
            {
                plain[plain_len++] = ch;
            }
            else if (ctx->state == PC_MAINT_RAW)
            {
                if (ch == '+' &&
                    ((!ctx->ever_received && now_ms >= PC_MAINT_PRE_GUARD_MS) ||
                     (ctx->ever_received &&
                      (uint32_t)(now_ms - ctx->last_byte_ms) >= PC_MAINT_PRE_GUARD_MS)))
                {
                    emit_raw(cb, plain, plain_len);
                    plain_len = 0U;
                    ctx->candidate[0] = ch;
                    ctx->length = 1U;
                    ctx->state = PC_MAINT_PLUS1;
                    ctx->stage_ms = now_ms;
                }
                else plain[plain_len++] = ch;
            }
            else if (ctx->state == PC_MAINT_PLUS1 || ctx->state == PC_MAINT_PLUS2)
            {
                if (ch == '+' &&
                    (uint32_t)(now_ms - ctx->stage_ms) <= PC_MAINT_PLUS_GAP_MS)
                {
                    ctx->candidate[ctx->length++] = ch;
                    ctx->stage_ms = now_ms;
                    ctx->state = (ctx->state == PC_MAINT_PLUS1) ? PC_MAINT_PLUS2 : PC_MAINT_POST;
                }
                else
                {
                    emit_raw(cb, plain, plain_len);
                    plain_len = 0U;
                    replay(ctx, cb);
                    retry = 1; /* reprocess current byte without loss */
                }
            }
            else if (ctx->state == PC_MAINT_POST)
            {
                emit_raw(cb, plain, plain_len);
                plain_len = 0U;
                replay(ctx, cb); /* any byte before post-guard is ordinary RAW */
                retry = 1;
            }
            else if (ctx->state == PC_MAINT_DRAIN)
            {
                ctx->session_activity_ms = now_ms;
                if (ch == '\r')
                {
                    ctx->state = PC_MAINT_SESSION;
                    ctx->length = 0U;
                    ctx->skip_next_lf = 1U;
                }
            }
            else if (ctx->state == PC_MAINT_CONFIRM || ctx->state == PC_MAINT_SESSION)
            {
                if (ctx->length == ((ctx->state == PC_MAINT_CONFIRM) ?
                                    PC_MAINT_CANDIDATE_MAX : PC_MAINT_COMMAND_MAX))
                {
                    if (ctx->state == PC_MAINT_CONFIRM)
                        replay(ctx, cb);
                    else
                    {
                        ctx->length = 0U; /* discard overlong local AT, never execute */
                        ctx->state = PC_MAINT_DRAIN;
                    }
                    retry = (ctx->state == PC_MAINT_RAW);
                }
                else
                {
                    ctx->candidate[ctx->length++] = ch;
                    if (ctx->state == PC_MAINT_SESSION)
                        ctx->session_activity_ms = now_ms;
                    if (ch == '\r')
                    {
                        size_t offset = (ctx->state == PC_MAINT_CONFIRM) ? 3U : 0U;
                        size_t cmd_len = ctx->length - offset - 1U;
                        if (ctx->state == PC_MAINT_CONFIRM &&
                            cmd_len == sizeof("AT+MCUCFG=1") - 1U &&
                            memcmp(ctx->candidate + offset, "AT+MCUCFG=1", cmd_len) == 0)
                        {
                            ctx->state = PC_MAINT_SESSION;
                            ctx->session_activity_ms = now_ms;
                            if (cb->command != NULL)
                                cb->command(cb->user, ctx->candidate + offset, cmd_len);
                            ctx->length = 0U;
                            ctx->skip_next_lf = 1U;
                        }
                        else if (ctx->state == PC_MAINT_CONFIRM)
                        {
                            emit_raw(cb, plain, plain_len);
                            plain_len = 0U;
                            replay(ctx, cb);
                        }
                        else
                        {
                            int exit = (cmd_len == sizeof("AT+MCUCFG=0") - 1U &&
                                        memcmp(ctx->candidate, "AT+MCUCFG=0", cmd_len) == 0);
                            if (cb->command != NULL)
                                cb->command(cb->user, ctx->candidate, cmd_len);
                            ctx->length = 0U;
                            ctx->skip_next_lf = 1U;
                            if (exit) ctx->state = PC_MAINT_RAW;
                        }
                    }
                }
            }
        }
        ctx->last_byte_ms = now_ms;
        ctx->ever_received = 1U;
        if (plain_len == sizeof(plain))
        {
            emit_raw(cb, plain, plain_len);
            plain_len = 0U;
        }
    }
    emit_raw(cb, plain, plain_len);
}

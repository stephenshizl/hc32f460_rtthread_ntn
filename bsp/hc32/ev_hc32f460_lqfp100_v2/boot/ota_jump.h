/* Copyright (c) 2026, youyoutech.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef NTN_OTA_JUMP_H
#define NTN_OTA_JUMP_H

#include "hc32_ll.h"

static bool ntn_ota_vector_valid(uint32_t vector, uint32_t end)
{
    const volatile uint32_t *words = (const volatile uint32_t *)vector;
    uint32_t stack = words[0];
    uint32_t entry = words[1];

    return (stack & 7U) == 0 && stack > 0x1FFF8018U && stack <= 0x20027000U &&
           (entry & 1U) != 0 && (entry & ~1UL) >= vector && (entry & ~1UL) < end;
}

/* Reached from privileged Thread mode after peripheral shutdown. */
#ifdef __clang__
static __attribute__((naked, noreturn)) void ntn_ota_branch(uint32_t stack, uint32_t entry)
{
    __asm volatile("movs r2, #0\n"
                   "msr control, r2\n"
                   "msr basepri, r2\n"
                   "msr faultmask, r2\n"
                   "msr msp, r0\n"
                   "isb\n"
                   "bx r1");
}
#else
static __asm void ntn_ota_branch(uint32_t stack, uint32_t entry)
{
    MOVS r2, #0
    MSR CONTROL, r2
    MSR BASEPRI, r2
    MSR FAULTMASK, r2
    MSR MSP, r0
    ISB
    BX r1
}
#endif

static void ntn_ota_jump(uint32_t vector)
{
    const volatile uint32_t *words = (const volatile uint32_t *)vector;
    uint32_t groups = (SCnSCB->ICTR & SCnSCB_ICTR_INTLINESNUM_Msk) + 1U;

    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    for (uint32_t i = 0; i < groups; ++i)
    {
        NVIC->ICER[i] = UINT32_MAX;
        NVIC->ICPR[i] = UINT32_MAX;
    }
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
    SCB->VTOR = vector;
    __DSB();
    __ISB();
    ntn_ota_branch(words[0], words[1]);
}

#endif

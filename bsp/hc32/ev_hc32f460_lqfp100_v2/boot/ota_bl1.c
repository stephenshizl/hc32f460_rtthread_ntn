/* Copyright (c) 2026, youyoutech.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <stdint.h>
#include "ota_layout.h"
#include "ota_jump.h"
#include "ota_bl2_digest.h"
#include "ota_watchdog.h"

static const uint8_t expected[32] = NTN_OTA_BL2_DIGEST;

static bool wait_tx(uint32_t flag)
{
    for (uint32_t retry = 0; retry < 500000U; ++retry)
        if (USART_GetStatus(CM_USART1, flag) == SET)
            return true;
    return false;
}

/* PA1 is the existing PC RS232 transmit line. This path runs only on failure. */
static void report_failure(const char *message)
{
    stc_usart_uart_init_t config;
    SystemCoreClockUpdate();
    GPIO_REG_Unlock();
    GPIO_SetFunc(GPIO_PORT_A, GPIO_PIN_01, GPIO_FUNC_32);
    GPIO_REG_Lock();
    PWC_FCG0_REG_Unlock();
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART1, ENABLE);
    PWC_FCG0_REG_Lock();
    if (USART_UART_StructInit(&config) != LL_OK)
        return;
    config.u32Baudrate = 19200U;
    if (USART_UART_Init(CM_USART1, &config, NULL) != LL_OK)
        return;
    USART_FuncCmd(CM_USART1, USART_TX, ENABLE);
    while (*message && wait_tx(USART_FLAG_TX_EMPTY))
        USART_WriteData(CM_USART1, (uint8_t)*message++);
    (void)wait_tx(USART_FLAG_TX_CPLT);
}

int main(void)
{
    uint32_t digest[8];
    const uint8_t *actual = (const uint8_t *)digest;
    volatile uint8_t mismatch = 0;
    int result;

    __disable_irq();
    ntn_ota_feed_watchdog();
    PWC_FCG0_REG_Unlock();
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_HASH, ENABLE);
    result = HASH_Calculate((const uint8_t *)NTN_OTA_BL2_BASE,
                            NTN_OTA_BL2_SIZE, (uint8_t *)digest);
    HASH_DeInit();
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_HASH, DISABLE);
    PWC_FCG0_REG_Lock();
    if (result == LL_OK)
    {
        for (uint32_t i = 0; i < sizeof(expected); ++i)
            mismatch |= actual[i] ^ expected[i];
        if (mismatch == 0)
        {
            if (ntn_ota_vector_valid(NTN_OTA_BL2_BASE, NTN_OTA_BL2_BASE + NTN_OTA_BL2_SIZE))
            {
                ntn_ota_feed_watchdog();
                ntn_ota_jump(NTN_OTA_BL2_BASE);
            }
            report_failure("BOOT1 ERROR BL2_VECTOR\r\n");
        }
        else
            report_failure("BOOT1 ERROR BL2_SHA256\r\n");
    }
    else
        report_failure("BOOT1 ERROR HASH_ENGINE\r\n");
    for (;;)
        __WFI();
}

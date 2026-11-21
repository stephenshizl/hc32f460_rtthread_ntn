/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-28     CDT          first version
 * 2024-06-11     CDT          remove CLK_Delay for usb, as it is already included in ddl API
 */

#include "board.h"
#if defined(NTN_OTA_APP) || defined(NTN_OTA_BL2)
#include "ota_power.h"
#endif
#include "board_config.h"

/* unlock/lock peripheral */
#define EXAMPLE_PERIPH_WE (LL_PERIPH_GPIO | LL_PERIPH_EFM | LL_PERIPH_FCG | \
                           LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_SRAM | LL_PERIPH_LVD)
#define EXAMPLE_PERIPH_WP (LL_PERIPH_EFM | LL_PERIPH_FCG | LL_PERIPH_SRAM)

#define BSP_HRC_FREQ_16MHZ (16000000UL)
#define BSP_HRC_FREQ_20MHZ (20000000UL)

static board_clock_status_t s_clock_status = BOARD_CLOCK_STATUS_NOT_CONFIGURED;
static rt_bool_t s_usb_clock_ready = RT_FALSE;

/** System Base Configuration
*/
void SystemBase_Config(void)
{
#if defined(BSP_USING_ON_CHIP_FLASH_CACHE)
    EFM_CacheCmd(ENABLE);
#endif

    /*
     * Release JTAG pins to GPIO/multiplexed peripherals, keep SWD (PA13/PA14) for debugging:
     * - PB3  (JTDO)   -> NTN_MCU_STATUS (GPIO input)
     * - PA15 (JTDI)   -> MCU_WAKEUP_NTN (GPIO output)
     * - PB4  (NJTRST) -> MCU_UART3_TXD  (USART3_TX to NTN MS150)
     */
    GPIO_SetDebugPort(GPIO_PIN_TDO | GPIO_PIN_TDI | GPIO_PIN_TRST, DISABLE);

    /* Initialize SEGGER RTT device early before rt_console_set_device() in rt_hw_board_init */
    extern int rt_hw_rtt_init(void);
    rt_hw_rtt_init();
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{
    stc_clock_pll_init_t stcMpllInit;
    stc_clock_freq_t stcClockFreq;
    int32_t i32Ret;
    uint32_t u32PllN;
#if defined(BSP_USING_USBD) || defined(BSP_USING_USBH) || defined(RT_USING_CHERRYUSB)
    stc_clock_pllx_init_t stcUpllInit;
    uint32_t u32UpllN;
#endif
#if defined(BSP_RTC_USING_XTAL32) || defined(RT_USING_PM)
    stc_clock_xtal32_init_t stcXtal32Init;
#endif

    /* PCLK0, HCLK  Max 200MHz */
    /* PCLK1, PCLK4 Max 100MHz */
    /* PCLK2, PCLK3 Max 50MHz  */
    /* EX BUS Max 100MHz */
    CLK_SetClockDiv(CLK_BUS_CLK_ALL, (CLK_HCLK_DIV1 | CLK_EXCLK_DIV2 | CLK_PCLK0_DIV1 | CLK_PCLK1_DIV2 |
                                      CLK_PCLK2_DIV4 | CLK_PCLK3_DIV4 | CLK_PCLK4_DIV2));

    /*
     * This product has no external high-speed crystal on PH0/PH1.
     * HRC is selected by ICG as either 16 MHz (project default) or 20 MHz.
     * Keep the system running from HRC if MPLL setup cannot be verified.
     */
    i32Ret = CLK_HrcCmd(ENABLE);
    SystemCoreClockUpdate();
    if (LL_OK != i32Ret)
    {
        s_clock_status = BOARD_CLOCK_STATUS_HRC_START_FAILED;
        goto clock_fallback;
    }

    if (BSP_HRC_FREQ_16MHZ == HRC_VALUE)
    {
        /* (16 MHz / 2) * 50 = 400 MHz VCO; /2 = 200 MHz. */
        u32PllN = 50UL;
    }
    else if (BSP_HRC_FREQ_20MHZ == HRC_VALUE)
    {
        /* (20 MHz / 2) * 40 = 400 MHz VCO; /2 = 200 MHz. */
        u32PllN = 40UL;
    }
    else
    {
        s_clock_status = BOARD_CLOCK_STATUS_HRC_FREQ_UNSUPPORTED;
        goto clock_fallback;
    }

    (void)CLK_PLLStructInit(&stcMpllInit);
    /* MPLL source is internal HRC; VCO=400 MHz, P/Q/R=200 MHz. */
    stcMpllInit.u8PLLState = CLK_PLL_ON;
    stcMpllInit.PLLCFGR = 0UL;
    stcMpllInit.PLLCFGR_f.PLLM = 2UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLN = u32PllN - 1UL;
    stcMpllInit.PLLCFGR_f.PLLP = 2UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLQ = 2UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLR = 2UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLSRC = CLK_PLL_SRC_HRC;
    i32Ret = CLK_PLLInit(&stcMpllInit);
    if (LL_OK != i32Ret)
    {
        s_clock_status = BOARD_CLOCK_STATUS_PLL_START_FAILED;
        goto clock_fallback;
    }

    /* sram init include read/write wait cycle setting */
    SRAM_SetWaitCycle(SRAM_SRAMH, SRAM_WAIT_CYCLE0, SRAM_WAIT_CYCLE0);
    SRAM_SetWaitCycle((SRAM_SRAM12 | SRAM_SRAM3 | SRAM_SRAMR), SRAM_WAIT_CYCLE1, SRAM_WAIT_CYCLE1);

    /* flash read wait cycle setting */
    (void)EFM_SetWaitCycle(EFM_WAIT_CYCLE5);
    /* 3 cycles for 126MHz ~ 200MHz */
    GPIO_SetReadWaitCycle(GPIO_RD_WAIT3);
    /* Switch driver ability */
    i32Ret = PWC_HighSpeedToHighPerformance();
    if (LL_OK != i32Ret)
    {
        s_clock_status = BOARD_CLOCK_STATUS_POWER_MODE_FAILED;
        goto clock_fallback;
    }
    /* Switch system clock source to MPLL. */
    CLK_SetSysClockSrc(CLK_SYSCLK_SRC_PLL);

    (void)CLK_GetClockFreq(&stcClockFreq);
    if (200000000UL != stcClockFreq.u32HclkFreq)
    {
        s_clock_status = BOARD_CLOCK_STATUS_PLL_VERIFY_FAILED;
        goto clock_fallback;
    }
    s_clock_status = BOARD_CLOCK_STATUS_PLL_200M;

#if defined(BSP_USING_USBD) || defined(BSP_USING_USBH) || defined(RT_USING_CHERRYUSB)
    /* UPLL shares the HRC source with MPLL. Generate 480 MHz VCO / 10 = 48 MHz. */
    u32UpllN = (BSP_HRC_FREQ_16MHZ == HRC_VALUE) ? 60UL : 48UL;
    (void)CLK_PLLxStructInit(&stcUpllInit);
    stcUpllInit.u8PLLState = CLK_PLL_ON;
    stcUpllInit.PLLCFGR = 0UL;
    stcUpllInit.PLLCFGR_f.PLLM = 2UL - 1UL;
    stcUpllInit.PLLCFGR_f.PLLN = u32UpllN - 1UL;
    stcUpllInit.PLLCFGR_f.PLLP = 10UL - 1UL;
    stcUpllInit.PLLCFGR_f.PLLQ = 6UL - 1UL;
    stcUpllInit.PLLCFGR_f.PLLR = 6UL - 1UL;
    if (LL_OK == CLK_PLLxInit(&stcUpllInit))
    {
        s_usb_clock_ready = RT_TRUE;
    }
#endif

#if defined(BSP_RTC_USING_XTAL32) || defined(RT_USING_PM)
    /* Xtal32 config */
    GPIO_AnalogCmd(XTAL32_PORT, XTAL32_IN_PIN | XTAL32_OUT_PIN, ENABLE);
    (void)CLK_Xtal32StructInit(&stcXtal32Init);
    stcXtal32Init.u8State = CLK_XTAL32_ON;
    stcXtal32Init.u8Drv = CLK_XTAL32_DRV_HIGH;
    stcXtal32Init.u8Filter = CLK_XTAL32_FILTER_RUN_MD;
    (void)CLK_Xtal32Init(&stcXtal32Init);
#endif

    return;

clock_fallback:
    /* HRC is the reset/default source and keeps RTT, SysTick, UART and SPI usable. */
    CLK_SetSysClockSrc(CLK_SYSCLK_SRC_HRC);
    (void)CLK_PLLCmd(DISABLE);
    SystemCoreClockUpdate();
}

board_clock_status_t board_clock_get_status(void)
{
    return s_clock_status;
}

rt_bool_t board_usb_clock_is_ready(void)
{
    return s_usb_clock_ready;
}

/** Peripheral Clock Configuration
*/
void PeripheralClock_Config(void)
{
#if defined(RT_USING_ADC)
    CLK_SetPeriClockSrc(CLK_PERIPHCLK_PCLK);
#endif

#if defined(BSP_USING_USBD) || defined(BSP_USING_USBH) || defined(RT_USING_CHERRYUSB)
    if (board_usb_clock_is_ready())
    {
        CLK_SetUSBClockSrc(CLK_USBCLK_PLLXP);
    }
#endif

#if defined(NTN_OTA_APP) || defined(NTN_OTA_BL2)
    /* Initialize the voltage monitor before RT-Thread device and storage init. */
    (void)ntn_ota_power_init();
    /* Seal on-chip Flash write window (end < start => all-deny) and lock EFM. */
    EFM_FWMC_Cmd(DISABLE);
    EFM_SetWindowProtectAddr(0x0007E000UL, 0x00000000UL);
    LL_PERIPH_WP(LL_PERIPH_EFM);
#endif
}

/** Peripheral Registers Unlock
*/
void PeripheralRegister_Unlock(void)
{
    LL_PERIPH_WE(EXAMPLE_PERIPH_WE);
}

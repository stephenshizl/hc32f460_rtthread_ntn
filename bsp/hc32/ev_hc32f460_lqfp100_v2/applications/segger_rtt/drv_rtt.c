/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-13     Stephen      First version: SEGGER RTT Console & Log driver for HC32F460
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "SEGGER_RTT.h"

#if defined(RT_USING_FINSH)
#include <finsh.h>
#endif

static struct rt_device _rtt_dev;
static rt_bool_t        _rtt_inited = RT_FALSE;

#if defined(RT_USING_FINSH)
static struct rt_timer  _rtt_rx_timer;
#endif

/**
 * @brief Console output callback before device driver framework is initialized
 *        or when console device is NULL. Overrides weak definition in kservice.c.
 */
void rt_hw_console_output(const char *str)
{
    if (str == RT_NULL)
    {
        return;
    }

    const char *p = str;
    while (*p != '\0')
    {
        const char *start = p;
        while (*p != '\0' && *p != '\n')
        {
            p++;
        }

        if (p > start)
        {
            SEGGER_RTT_Write(0, start, (unsigned)(p - start));
        }

        if (*p == '\n')
        {
            if (p > start && *(p - 1) == '\r')
            {
                static const char lf = '\n';
                SEGGER_RTT_Write(0, &lf, 1);
            }
            else
            {
                static const char crlf[2] = {'\r', '\n'};
                SEGGER_RTT_Write(0, crlf, 2);
            }
            p++;
        }
    }
}

static rt_err_t rtt_open(rt_device_t dev, rt_uint16_t oflag)
{
    if (dev != RT_NULL)
    {
        dev->open_flag = oflag;
    }
    return RT_EOK;
}

static rt_err_t rtt_close(rt_device_t dev)
{
    RT_UNUSED(dev);
    return RT_EOK;
}

static rt_ssize_t rtt_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    RT_UNUSED(dev);
    RT_UNUSED(pos);

    if (buffer == RT_NULL || size == 0)
    {
        return 0;
    }

    return (rt_ssize_t)SEGGER_RTT_Read(0, buffer, size);
}

static rt_ssize_t rtt_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    RT_UNUSED(pos);

    if (buffer == RT_NULL || size == 0)
    {
        return 0;
    }

    /* If stream mode is enabled, ensure newline has carriage return */
    if (dev != RT_NULL && (dev->open_flag & RT_DEVICE_FLAG_STREAM))
    {
        const char *p = (const char *)buffer;
        rt_size_t remaining = size;

        while (remaining > 0)
        {
            rt_size_t len = 0;
            while (len < remaining && p[len] != '\n')
            {
                len++;
            }

            if (len > 0)
            {
                SEGGER_RTT_Write(0, p, (unsigned)len);
                p += len;
                remaining -= len;
            }

            if (remaining > 0 && *p == '\n')
            {
                if (p > (const char *)buffer && *(p - 1) == '\r')
                {
                    static const char lf = '\n';
                    SEGGER_RTT_Write(0, &lf, 1);
                }
                else
                {
                    static const char crlf[2] = {'\r', '\n'};
                    SEGGER_RTT_Write(0, crlf, 2);
                }
                p++;
                remaining--;
            }
        }
        return (rt_ssize_t)size;
    }

    return (rt_ssize_t)SEGGER_RTT_Write(0, buffer, (unsigned)size);
}

static rt_err_t rtt_control(rt_device_t dev, int cmd, void *args)
{
    RT_UNUSED(dev);

    if (cmd == RT_DEVICE_CTRL_CONSOLE_OFLAG)
    {
        if (args != RT_NULL)
        {
            *(rt_uint16_t *)args = RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM;
        }
    }

    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops _rtt_ops =
{
    RT_NULL,
    rtt_open,
    rtt_close,
    rtt_read,
    rtt_write,
    rtt_control
};
#endif

#if defined(RT_USING_FINSH)
static void rtt_rx_timer_callback(void *parameter)
{
    RT_UNUSED(parameter);

    if (SEGGER_RTT_HasKey())
    {
        if (_rtt_dev.rx_indicate != RT_NULL)
        {
            _rtt_dev.rx_indicate(&_rtt_dev, 1);
        }
    }
}
#endif

/**
 * @brief Initialize SEGGER RTT device and register to RT-Thread device system
 */
int rt_hw_rtt_init(void)
{
    rt_err_t result;

    if (_rtt_inited)
    {
        return RT_EOK;
    }

    /* Initialize SEGGER RTT Core */
    SEGGER_RTT_Init();
    SEGGER_RTT_ConfigUpBuffer(0, "Terminal", NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    /* Register rt_device */
    _rtt_dev.type        = RT_Device_Class_Char;
    _rtt_dev.rx_indicate = RT_NULL;
    _rtt_dev.tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    _rtt_dev.ops         = &_rtt_ops;
#else
    _rtt_dev.init        = RT_NULL;
    _rtt_dev.open        = rtt_open;
    _rtt_dev.close       = rtt_close;
    _rtt_dev.read        = rtt_read;
    _rtt_dev.write       = rtt_write;
    _rtt_dev.control     = rtt_control;
#endif

    _rtt_dev.user_data   = RT_NULL;

    result = rt_device_register(&_rtt_dev, "segger", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STREAM);
    if (result != RT_EOK)
    {
        return result;
    }

    /* Switch console device to SEGGER RTT */
    rt_console_set_device("segger");

    _rtt_inited = RT_TRUE;
    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_rtt_init);

/**
 * @brief Bind FinSH shell and print startup banner
 */
int rt_hw_rtt_app_init(void)
{
    stc_clock_freq_t clock_freq;

#if defined(RT_USING_FINSH)
    /* FinSH binds the console in its thread after finsh_system_init().
     * APP init ordering does not guarantee that shell exists here. */

    /* Create and start RX check timer for interactive MSH shell (20ms interval) */
    rt_timer_init(&_rtt_rx_timer, "rtt_rx", rtt_rx_timer_callback, RT_NULL,
                  rt_tick_from_millisecond(20),
                  RT_TIMER_FLAG_PERIODIC);
    rt_timer_start(&_rtt_rx_timer);
#endif

    /* Print startup banner via RTT */
    rt_kprintf("\r\n");
    rt_kprintf("==============================================================\r\n");
    rt_kprintf("  Iridium NTN Smart Terminal - SEGGER RTT Console & Log Ready \r\n");
    rt_kprintf("  MCU: Xiaohua HC32F460 (ARM Cortex-M4)                       \r\n");
    rt_kprintf("  Console Device: segger (Up: 2048B, Down: 128B)              \r\n");
    rt_kprintf("  Interactive Shell: FinSH MSH Enabled via SWD RTT            \r\n");
    rt_kprintf("==============================================================\r\n\r\n");

    (void)CLK_GetClockFreq(&clock_freq);
    rt_kprintf("[CLK] status=%d, HRC=%u Hz, HCLK=%u Hz, PCLK1=%u Hz, PCLK2=%u Hz\r\n",
               board_clock_get_status(), HRC_VALUE, clock_freq.u32HclkFreq,
               clock_freq.u32Pclk1Freq, clock_freq.u32Pclk2Freq);

    return RT_EOK;
}
INIT_APP_EXPORT(rt_hw_rtt_app_init);

int SEGGER_RTT_vprintf(unsigned BufferIndex, const char * sFormat, va_list * pParamList)
{
    char acBuffer[128];
    int r;
    unsigned write_len;

    r = rt_vsnprintf(acBuffer, sizeof(acBuffer), sFormat, *pParamList);
    if (r > 0)
    {
        write_len = (r >= (int)sizeof(acBuffer)) ? (sizeof(acBuffer) - 1U) : (unsigned)r;
        SEGGER_RTT_Write(BufferIndex, acBuffer, write_len);
    }
    return r;
}

#if defined(RT_USING_FINSH)
static int clock_status(int argc, char **argv)
{
    stc_clock_freq_t clock_freq;

    RT_UNUSED(argc);
    RT_UNUSED(argv);
    (void)CLK_GetClockFreq(&clock_freq);

    rt_kprintf("clock_status=%d (0=HRC->MPLL 200MHz)\r\n", board_clock_get_status());
    rt_kprintf("HRC=%u HCLK=%u PCLK0=%u PCLK1=%u PCLK2=%u PCLK3=%u PCLK4=%u\r\n",
               HRC_VALUE, clock_freq.u32HclkFreq, clock_freq.u32Pclk0Freq,
               clock_freq.u32Pclk1Freq, clock_freq.u32Pclk2Freq,
               clock_freq.u32Pclk3Freq, clock_freq.u32Pclk4Freq);
    return 0;
}
MSH_CMD_EXPORT(clock_status, show HRC PLL and peripheral bus clocks);
#endif

int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...)
{
    va_list ParamList;
    int r;

    va_start(ParamList, sFormat);
    r = SEGGER_RTT_vprintf(BufferIndex, sFormat, &ParamList);
    va_end(ParamList);

    return r;
}

#if defined(RT_USING_FINSH)
static int rtt_test(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    rt_kprintf("[RTT Test] rt_kprintf working over SEGGER RTT console!\r\n");
    SEGGER_RTT_printf(0, "[RTT Test] Direct SEGGER_RTT_printf channel 0 output test: OK!\r\n");
    return 0;
}
MSH_CMD_EXPORT(rtt_test, test SEGGER RTT console and log output);
#endif

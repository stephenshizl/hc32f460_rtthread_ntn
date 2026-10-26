/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*       (c) 2014 - 2017  SEGGER Microcontroller GmbH & Co. KG        *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
*       SEGGER RTT * Real Time Transfer for embedded targets         *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* SEGGER strongly recommends to not make any changes                 *
* to or modify the source code of this software in order to stay     *
* compatible with the RTT protocol and J-Link.                       *
*                                                                    *
**********************************************************************
*                                                                    *
*       RTT version: 6.22d                                           *
*                                                                    *
**********************************************************************
---------------------------END-OF-HEADER------------------------------
File    : SEGGER_RTT_Conf.h
Purpose : Configuration for SEGGER Real Time Transfer on RT-Thread
----------------------------------------------------------------------
*/

#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

#include <rtthread.h>

#define SEGGER_RTT_MAX_NUM_UP_BUFFERS             (3)     // Max. number of up-buffers (T->H)
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS           (3)     // Max. number of down-buffers (H->T)

#define BUFFER_SIZE_UP                            (8192)  // Full shell help burst; keep non-blocking output
#define BUFFER_SIZE_DOWN                          (128)   // Buffer size for terminal input (128B)

#define SEGGER_RTT_PRINTF_BUFFER_SIZE             (64u)   // Buffer for RTT printf

#define SEGGER_RTT_MODE_DEFAULT                   SEGGER_RTT_MODE_NO_BLOCK_SKIP // Non-blocking skip when buffer full

#define USE_RTT_ASM                               (0)

#define SEGGER_RTT_MEMCPY_USE_BYTELOOP            (0)

#define SEGGER_RTT_MAX_INTERRUPT_PRIORITY         (0x20)

/* RT-Thread critical section lock/unlock for RTT operations */
#ifndef SEGGER_RTT_LOCK
  #define SEGGER_RTT_LOCK()                         { rt_base_t _rtt_level = rt_hw_interrupt_disable();
#endif

#ifndef SEGGER_RTT_UNLOCK
  #define SEGGER_RTT_UNLOCK()                       rt_hw_interrupt_enable(_rtt_level); }
#endif

#endif /* SEGGER_RTT_CONF_H */
/*************************** End of file ****************************/

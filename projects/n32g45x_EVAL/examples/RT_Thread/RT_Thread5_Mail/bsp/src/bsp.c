/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file bsp.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include <rthw.h>
#include <rtthread.h>

#include "n32g45x.h"
#include "bsp.h"
#include "log.h"

/**
 * @brief  Configures Vector Table base location.
 */
void NVIC_Configuration(void)
{
#ifdef  VECT_TAB_RAM
    /* Set the Vector Table base location at 0x20000000 */
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
    /* Set the Vector Table base location at 0x08000000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif
}

#ifdef PRINT_RCC_FREQ_INFO
/**
 * @brief print RCC freq information
 */
void print_rcc_freq_info(void)
{
    RCC_ClocksType RCC_ClockFreq;

    RCC_GetClocksFreqValue(&RCC_ClockFreq);

    rt_kprintf("\nSYSCLK_Frequency is %dHZ", RCC_ClockFreq.SysclkFreq);
    rt_kprintf("\nPCLK1_Frequency is %dHZ", RCC_ClockFreq.Pclk1Freq);
    rt_kprintf("\nPCLK2_Frequency is %dHZ", RCC_ClockFreq.Pclk2Freq);
    rt_kprintf("\nHCLK_Frequency is %dHZ", RCC_ClockFreq.HclkFreq);

    rt_kprintf("\nADCPLLCLK_Frequency is %dHZ", RCC_ClockFreq.AdcPllClkFreq);
    rt_kprintf("\nADCHCLK_Frequency is %dHZ", RCC_ClockFreq.AdcHclkFreq);

    rt_kprintf("\nSystemCoreClock is %dHZ\n", SystemCoreClock);
}
#endif

/**
 * @brief This is the timer interrupt service routine.
 */
void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

/**
 * @brief This function will initial N32G45x board.
 */
void rt_hw_board_init()
{
    /* NVIC Configuration */
    NVIC_Configuration();

    /* Configure the SysTick */
    SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);   /* 10ms */

    /* Initial usart deriver, and set console device */
    log_init();
#ifdef RT_USING_CONSOLE
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif
    /* Print RCC freq info */
#ifdef PRINT_RCC_FREQ_INFO
    print_rcc_freq_info();
#endif

}

/*@}*/

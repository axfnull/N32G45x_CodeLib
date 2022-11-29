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
 * @file main.c
 * @author Nations 
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include "log.h"
#include "n32g45x_rtc.h"
#include "n32g45x.h"
#include "User_RTC_Config.h"
#include "User_LED_Config.h"

/** @addtogroup RTC_Calendar
 * @{
 */

/**
 * @brief  Configure the system tick clock.
 */
uint32_t DBG_SysTick_Config(uint32_t ticks)
{
    if (ticks > SysTick_LOAD_RELOAD_Msk)
        return (1); /* Reload value impossible */
    SysTick->LOAD = (ticks & SysTick_LOAD_RELOAD_Msk) - 1;       /* set reload register */
    NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1); /* set Priority for Cortex-M0 System Interrupts */
    SysTick->VAL  = 0;                                           /* Load the SysTick Counter Value */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; /* Enable SysTick IRQ and SysTick Timer */
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    return (0); /* Function successful */
}


/**
 * @brief  Use the system tick clock to delay.
 */
void SysTick_Delay_Ms(__IO uint32_t ms)
{
    uint32_t i;
    RCC_ClocksType RCC_Clocks;
    RCC_GetClocksFreqValue(&RCC_Clocks);
    DBG_SysTick_Config(RCC_Clocks.SysclkFreq / 1000);
    for (i = 0; i < ms; i++)
    {
       /* When the value of the counter is reduced to 0, the bit 16 of the CRTL register is set to 1.
          When set to 1, the bit  will cleared to 0 with read
       */
       while (!((SysTick->CTRL) & (1 << 16)));
    }
    // Disable SysTick timer
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

/**
 * @brief  Main program.
 */
int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured,
         this is done through SystemInit() function which is called from startup
         file (startup_n32g45x_xx.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_n32g45x.c file
       */
    /* Initialize LEDs on n32g45x-EVAL board */
    LEDInit(LED1_PORT,LED1_PIN);
    LEDOff(LED1_PORT,LED1_PIN);
    /* Initialize USART,TX: PC10 RX: PC11*/
    log_init();
    log_info("RTC Init");
    /* RTC date and time default value*/
    RTC_DateAndTimeDefaultVale();
    /* RTC clock source select */
    RTC_CLKSourceConfig(RTC_CLK_SRC_TYPE_LSE, true, true);
    RTC_PrescalerConfig();
    /* Adjust time by values entered by the user on the hyperterminal */
    RTC_DateRegulate();
    RTC_TimeRegulate();
    /* Adjust time by values entered by the user on the hyperterminal */
    RTC_ConfigCalibOutput(RTC_CALIB_OUTPUT_1HZ);
    /* Calibrate output config,push pull */
    RTC_ConfigOutputType(RTC_OUTPUT_PUSHPULL);
    /* Calibrate output enable*/
    RTC_EnableCalibOutput(ENABLE);
    /* Configure EXTI PD8 pin  connected to RTC TimeStamp
    (while externally feeding PD8 with 1HZ signal output from PC13)*/
    EXTI_PB8_TimeStamp_Configuration();
    EXTI20_TimeStampIRQn_Configuration(EXTI_Trigger_Falling);
      /* clear RTC time stamp flag  */
    RTC_ClrFlag(RTC_FLAG_TISF);
    RTC_ClrFlag(RTC_FLAG_TISOVF);
    RTC_EnableTimeStamp(RTC_TIMESTAMP_EDGE_FALLING, ENABLE);
    while (1)
    {
       if (RTC_GetFlagStatus(RTC_FLAG_TISF) != RESET)
       {
          /* Every interval delay (1s) displays calendar and clock stamp register contents*/
          RTC_DateShow();
          RTC_TimeShow();
          RTC_TimeStampShow();
          RTC_ClrFlag(RTC_FLAG_TISF);
          RTC_ClrFlag(RTC_FLAG_TISOVF);
       }
       SysTick_Delay_Ms(1000);
    }
}

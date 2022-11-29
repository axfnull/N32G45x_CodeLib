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
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include <stdio.h>
#include <stdint.h>
#include "n32g45x.h"
#include "main.h"
#include "User_LED_Config.h"
#include "log.h"

/** @addtogroup PWR_STANDBY
 * @{
 */

/**
 * @brief  Delay.
 * @param  set the delay time .
 *   This parameter can be one of following parameters:
 *     @arg nCount
 */
void Delay(u32 nCount)
{
    u32 index = 0;
    for (index = (34000 * nCount); index != 0; index--)
    {
    }
}
/**
 * @brief  Main program.
 */
int main(void)
{
    /*Clean backup flag to exit debug mode */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR,ENABLE);
    PWR_BackupAccessEnable(ENABLE);
    RCC_EnableBackupReset(DISABLE);
    log_init();
    log_info("\r\n MCU Reset \r\n");
    /* Initialize LEDs */
    LEDInit(LED1_PORT, LED1_PIN);
    LEDInit(LED3_PORT, LED3_PIN);
    /* Turn on LED1 */
    LEDOn(LED1_PORT, LED1_PIN);
    LEDOn(LED3_PORT, LED3_PIN);
    /* Enable PWR and BKP clock */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
    /* Enable WKUP pin */
    PWR_WakeUpPinEnable(ENABLE);
    while (1)
    {
       /* Check if the Wake-Up flag is set */
       if (PWR_GetFlagStatus(PWR_WU_FLAG) != RESET)
       {
          /* Clear Wake Up flag */
          PWR_ClearFlag(PWR_WU_FLAG);
       }
       /* Delay a long time */
       Delay(600);
       /* Request to enter STANDBY mode */
       PWR_EnterStandbyState();
    }
}

/**
 * @}
 */

/**
 * @}
 */

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
 * @file User_LED_Config.c
 * @author Nations 
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */


#include "n32g45x.h"
#include "User_LED_Config.h"

/**
 * @brief  Toggles the selected led.
 * @param GPIOx Specifies the led port to be toggled.
 *   This parameter can be one of following parameters:
 *     @arg LED1_PORT
 *     @arg LED2_PORT
 *     @arg LED3_PORT
 * @param Led Specifies the led to be toggled.
 *   This parameter can be one of following parameters:
 *     @arg LED1
 *     @arg LED2
 *     @arg LED3
 */
void LEDBlink(GPIO_Module* GPIOx,uint16_t Pin)
{
    GPIOx->POD ^= Pin;
}

/**
 * @brief  Turns selected Led on.
 * @param GPIOx Specifies the led port to be set on.
 *   This parameter can be one of following parameters:
 *     @arg LED1_PORT
 *     @arg LED2_PORT
 *     @arg LED3_PORT
 * @param Led Specifies the Led to be set on.
 *   This parameter can be one of following parameters:
 *     @arg LED1
 *     @arg LED2
 *     @arg LED3
 */
void LEDOn(GPIO_Module* GPIOx,uint16_t Pin)
{
    GPIOx->PBSC = Pin;
}

/**
 * @brief  Turns selected led Off.
 * @param GPIOx Specifies the led port to be set off.
 *   This parameter can be one of following parameters:
 *     @arg LED1_PORT
 *     @arg LED2_PORT
 *     @arg LED3_PORT
 * @param Led Specifies the Led to be set off.
 *   This parameter can be one of following parameters:
 *     @arg LED1
 *     @arg LED2
 *     @arg LED3
 */
void LEDOff(GPIO_Module* GPIOx,uint16_t Pin)
{
    GPIOx->PBC = Pin;
}

/**
 * @brief config the led pin .
 * @param GPIOx Specifies the led port to be configured.
 *   This parameter can be one of following parameters:
 *     @arg LED1_PORT
 *     @arg LED2_PORT
 *     @arg LED3_PORT
 * @param Pin Specifies the led pin to be configured.
 *   This parameter can be one of following parameters:
 *     @arg LED1
 *     @arg LED2
 *     @arg LED3
 */
void LEDInit(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStruct(&GPIO_InitStructure);
    /* Enable LED Port Clock */
    if(GPIOA==GPIOx)
    {
      RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    }
    else if(GPIOB==GPIOx)
    {
      RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    }
    else if(GPIOC==GPIOx)
    {
      RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC, ENABLE);
    }
    else if(GPIOD==GPIOx)
    {
      RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOD, ENABLE);
    }
    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.Pin        = Pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);
}

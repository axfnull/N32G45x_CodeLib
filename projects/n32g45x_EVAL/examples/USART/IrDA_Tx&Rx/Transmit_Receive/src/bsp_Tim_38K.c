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
 * @file bsp_Tim_38K.c
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include "bsp_Tim_38K.h"



/**
 * @brief  Configures the different system clocks.
 */
void ADVANCE_TIM_RCC_Configuration(void)
{
    /* TIM1 clock enable */
    RCC_EnableAPB2PeriphClk(ADVANCE_TIM_CLK, ENABLE);
    /* GPIOC clock enable */
    RCC_EnableAPB2PeriphClk(ADVANCE_TIM_CH1_GPIO_CLK, ENABLE);
}
/**
 * @brief  Configure the nested vectored interrupt controller.
 */
void ADVANCE_TIM_NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;
    /* Enable the TIM1 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = ADVANCE_TIM_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/**
 * @brief  Configure the GPIO Pins.
 */
void ADVANCE_TIM_GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStructure.Pin        = ADVANCE_TIM_CH1_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(ADVANCE_TIM_CH1_PORT, &GPIO_InitStructure);
}
/**
 * @brief  Configures tim1 clocks.
 */
void ADVANCE_TIM_Configuration(void)
{
    uint16_t PrescalerValue = 0;
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);   
    /* Compute the prescaler value */
    PrescalerValue = 0;
    /* Time base configuration */
    TIM_TimeBaseStructure.Period    = ADVANCE_TIM_Period;
    TIM_TimeBaseStructure.Prescaler = ADVANCE_TIM_Prescaler;
    TIM_TimeBaseStructure.ClkDiv    = TIM_CLK_DIV1;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_InitTimeBase(ADVANCE_TIM, &TIM_TimeBaseStructure);
    /* Prescaler configuration */
    TIM_ConfigPrescaler(ADVANCE_TIM, PrescalerValue, TIM_PSC_RELOAD_MODE_IMMEDIATE);
    /* TIM1 enable update irq */
    TIM_ConfigInt(ADVANCE_TIM, TIM_INT_UPDATE, ENABLE);
    /* TIM1 enable counter */
    TIM_Enable(TIM1, ENABLE);
}
void ADVANCE_TIM_Init(void)
{
    /* System Clocks Configuration */
    ADVANCE_TIM_RCC_Configuration();
    /* NVIC Configuration */
    ADVANCE_TIM_NVIC_Configuration();
    /* GPIO Configuration */
    ADVANCE_TIM_GPIO_Configuration();
    /* TIM Configuration */
    ADVANCE_TIM_Configuration();
    
}


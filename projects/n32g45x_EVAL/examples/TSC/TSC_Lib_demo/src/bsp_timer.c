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
 * @file bsp_timer.c
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "bsp_timer.h"

/**
 * @brief Config a timer for 1ms time interrupt
 * @param void
 * @return void
 */
void TIM6_init(void)
{
    TIM_TimeBaseInitType TIM_TimeBaseStructure = {0};
    NVIC_InitType NVIC_InitStructure;

    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM6, ENABLE); /* Enable the clock */

    /* Timer init */
    TIM_TimeBaseStructure.Period    = 19;                /* Set the auto reload value */
    TIM_TimeBaseStructure.Prescaler = 3599;              /* Set the prescaler of tim */
    TIM_InitTimeBase(TIM6, &TIM_TimeBaseStructure);      /* Init the timer */

    /* Config interrupt */
    TIM_ClrIntPendingBit(TIM6,TIM_INT_UPDATE);      /*Clear timer update interrupt flag */
    TIM_ConfigInt(TIM6, TIM_INT_UPDATE, ENABLE);    /* Enable update interrupt */

    /* NVIC config */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM6_IRQn; /* TIM6 interrupt */
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;    /* enable interrupt channel */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;         /* pre priority = 2*/
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;         /* sub priority=0*/
    NVIC_Init(&NVIC_InitStructure);                                   /* NVIC config */

    /* Timer enable */
    TIM_Enable(TIM6, ENABLE);
}


/**
 * @brief  Config a timer for tsc analyze
 * @param  totals_ch specifies the total channels to be analyzed
 * @return void
 */
void TIM7_init(uint32_t totals_ch)
{
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    NVIC_InitType NVIC_InitStructure;

    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM7, ENABLE); /* Enalbe TIM7 clock */

    /* Timer init */
    TIM_TimeBaseStructure.Period = TSC_ALG_HANDLER_PERIOD_PER_CHN /* ms*/ / totals_ch - 1;
    TIM_TimeBaseStructure.Prescaler = 3599;         /* Set the prescaler of tim */
    TIM_InitTimeBase(TIM7, &TIM_TimeBaseStructure); /* TIM init */

    /* Config interrupt */
    TIM_ClrIntPendingBit(TIM7,TIM_INT_UPDATE);      /*Clear timer update interrupt flag */
    TIM_ConfigInt(TIM7, TIM_INT_UPDATE, ENABLE);    /* Enable update interrupt */

    /* NVIC config */
    NVIC_InitStructure.NVIC_IRQChannel    = TIM7_IRQn;          /* TIM7 interrupt */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             /* enable interrupt channel */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;   /* pre priority = 3*/
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;   /* sub priority = 2*/
    NVIC_Init(&NVIC_InitStructure);                             /* NVIC config */

    /* Timer disable */
    TIM_Enable(TIM7, DISABLE); 
}


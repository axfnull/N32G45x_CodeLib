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
 * @brief  Timer3 initialization configuration.
 */
void tim3_init(uint16_t Period, uint16_t Prescaler)
{
    TIM_TimeBaseInitType timInitStruct;
    NVIC_InitType NVIC_InitStructure;

    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, ENABLE);

    timInitStruct.Period    = Period;
    timInitStruct.Prescaler = Prescaler;
    timInitStruct.ClkDiv    = 0;
    timInitStruct.CntMode   = TIM_CNT_MODE_UP;
    TIM_InitTimeBase(TIM3, &timInitStruct);

    TIM_ConfigInt(TIM3, TIM_INT_UPDATE, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Enable(TIM3, ENABLE);
}


/***
 * @brief    Get ETR counts by counters of timer2.
 */
void TSC_EtrTimInit(TIM_Module *TIMx)
{
    TIM_TimeBaseInitType timInitStruct;

    if(TIM2 == TIMx)
        RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM2, ENABLE);
    else if(TIM4 == TIMx)
        RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM4, ENABLE);
    else
        return;
    
    // Reset timer to default.
    TIM_DeInit(TIMx);

    TIM_InitTimBaseStruct(&timInitStruct);
    timInitStruct.CapEtrSelFromTscEn = true;
    TIM_InitTimeBase(TIMx, &timInitStruct);

//    TIM_SelectOnePulseMode(TIMx,TIM_OPMODE_REPET);
//    TIM_EnableUpdateEvt(TIMx,DISABLE);
//    TIM_ConfigUpdateRequestIntSrc(TIMx,DISABLE);
    TIM_ConfigArPreload(TIMx,ENABLE);

    TIM_SelectMasterSlaveMode(TIMx,TIM_MASTER_SLAVE_MODE_ENABLE);
    TIM_SelectSlaveMode(TIMx,TIM_SLAVE_MODE_TRIG);
    TIM_ConfigExtClkMode2(  TIMx,
                            TIM_EXT_TRG_PSC_OFF,
                            TIM_EXT_TRIG_POLARITY_NONINVERTED,
                            0 );
    TIMx->CNT = 0;
    TIMx->STS = 0;
    TIM_Enable(TIMx, ENABLE);
}





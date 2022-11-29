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
        /* Enable DMA clocks */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_DMA1, ENABLE);
    /* TIM1 clock enable */
    RCC_EnableAPB2PeriphClk(ADVANCE_TIM_CLK, ENABLE);
    /* GPIOC clock enable */
    RCC_EnableAPB2PeriphClk(ADVANCE_TIM_CH1_GPIO_CLK, ENABLE);
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
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);   

    /* Time base configuration */
    TIM_TimeBaseStructure.Period    = ADVANCE_TIM_Period;
    TIM_TimeBaseStructure.Prescaler = ADVANCE_TIM_Prescaler;
    TIM_TimeBaseStructure.ClkDiv    = TIM_CLK_DIV1;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_InitTimeBase(ADVANCE_TIM, &TIM_TimeBaseStructure);
        TIM_EnableDma(ADVANCE_TIM, TIM_DMA_UPDATE, ENABLE);

    /* TIM1 enable counter */
    TIM_Enable(ADVANCE_TIM, ENABLE);
}

volatile uint32_t PinOutputValue_Px3[2]={(1<<3),(1<<19)};
//volatile uint32_t PinOutputValue_Px8[2]={(1<<8),(1<<24)};
void DMA_Configuration(void)
{
    DMA_InitType DMA_InitStructure;

    /* DMA1 Channel1 Config */
    DMA_DeInit(DMA1_CH1);

    DMA_InitStructure.PeriphAddr     = (uint32_t)&GPIOD->PBSC;
    DMA_InitStructure.MemAddr        = (uint32_t)&PinOutputValue_Px3;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.BufSize        = 2;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_WORD;
    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_Word;
    DMA_InitStructure.CircularMode   = DMA_MODE_CIRCULAR;
    DMA_InitStructure.Priority       = DMA_PRIORITY_VERY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;

    DMA_Init(DMA1_CH1, &DMA_InitStructure);
    DMA_RequestRemap(DMA1_REMAP_TIM1_UP, DMA1, DMA1_CH1, ENABLE);

    /* DMA1 Channel1 enable */
    DMA_EnableChannel(DMA1_CH1, ENABLE);
}
void ADVANCE_TIM_Init(void)
{
    /* System Clocks Configuration */
    ADVANCE_TIM_RCC_Configuration();
    /* GPIO Configuration */
    ADVANCE_TIM_GPIO_Configuration();
    /* DMA configuration ------------------------------------------------------*/
    DMA_Configuration();
      /* TIM Configuration */
    ADVANCE_TIM_Configuration();
    
}


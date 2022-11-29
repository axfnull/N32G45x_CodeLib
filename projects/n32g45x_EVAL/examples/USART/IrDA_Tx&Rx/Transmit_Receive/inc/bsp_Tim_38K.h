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
 * @file bsp_Tim_38K.h
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __BSP_TIM_38K_H__
#define __BSP_TIM_38K_H__

#include "n32g45x.h"

#define ADVANCE_TIM               TIM1
#define ADVANCE_TIM_APBxClock_FUN RCC_EnableAPB2PeriphClk
#define ADVANCE_TIM_CLK           RCC_APB2_PERIPH_TIM1 
#define ADVANCE_TIM_Period        (1895 - 1)
#define ADVANCE_TIM_Prescaler     (4 - 1)
#define ADVANCE_TIM_Pulse         (947)
#define ADVANCE_TIM_IRQn           TIM1_UP_IRQn 
#define ADVANCE_TIM_IRQHandler     TIM1_UP_IRQHandler

#define ADVANCE_TIM_CH1_GPIO_CLK    RCC_APB2_PERIPH_GPIOD
#define ADVANCE_TIM_CH1_PORT        GPIOD      
#define ADVANCE_TIM_CH1_PIN         GPIO_PIN_3
#define ADVANCE_TIM_CH1_GPIO_RMP    GPIO_ALL_RMP_TIM1

void ADVANCE_TIM_Init(void);
#endif /* __BSP_TIM_38K_H__ */

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
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "main.h"

/** @addtogroup TIM_ExtTrigger_Synchro
 * @{
 */

TIM_TimeBaseInitType TIM_TimeBaseStructure;
TIM_ICInitType TIM_ICInitStructure;
OCInitType TIM_OCInitStructure;

void RCC_Configuration(void);
void GPIO_Configuration(void);

/**
 * @brief   Main program
 */
int main(void)
{
    /* System Clocks Configuration */
    RCC_Configuration();

    /* Configure the GPIO ports */
    GPIO_Configuration();

    /* Timers synchronisation in cascade mode with an external trigger -----
    1/TIM1 is configured as Master Timer:
     - Toggle Mode is used
     - The TIM1 Enable event is used as Trigger Output

    2/TIM1 is configured as Slave Timer for an external Trigger connected
     to TIM1 TI2 pin (TIM1 CH2 configured as input pin):
     - The TIM1 TI2FP2 is used as Trigger Input
     - Rising edge is used to start and stop the TIM1: Gated Mode.

    3/TIM3 is slave for TIM1 and Master for TIM4,
     - Toggle Mode is used
     - The ITR1(TIM1) is used as input trigger
     - Gated mode is used, so start and stop of slave counter
       are controlled by the Master trigger output signal(TIM1 enable event).
     - The TIM3 enable event is used as Trigger Output.

    4/TIM4 is slave for TIM3,
     - Toggle Mode is used
     - The ITR2(TIM3) is used as input trigger
     - Gated mode is used, so start and stop of slave counter
       are controlled by the Master trigger output signal(TIM3 enable event).

    * For Low-density, Medium-density, High-density and Connectivity line devices:
      The TIMxCLK is fixed to 72 MHZ, the Prescaler is equal to 2 so the TIMx clock
      counter is equal to 24 MHz.
      The Three Timers are running at:
      TIMx frequency = TIMx clock counter/ 2*(TIMx_Period + 1) = 162.1 KHz.

    * For Low-Density Value line and Medium-Density Value line devices:
      The TIMxCLK is fixed to 24 MHz, the Prescaler is equal to 2 so the TIMx clock
      counter is equal to 8 MHz.
      TIMx frequency = TIMx clock counter/ 2*(TIMx_Period + 1) = 54 KHz.

    The starts and stops of the TIM1 counters are controlled by the
    external trigger.
    The TIM3 starts and stops are controlled by the TIM1, and the TIM4
    starts and stops are controlled by the TIM3.
    -------------------------------------------------------------------- */

    /* Time base configuration */
    TIM_TimeBaseStructure.Period    = 73;
    TIM_TimeBaseStructure.Prescaler = 2;
    TIM_TimeBaseStructure.ClkDiv    = 0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;

    TIM_InitTimeBase(TIM1, &TIM_TimeBaseStructure);

    TIM_TimeBaseStructure.Period = 73;
    TIM_InitTimeBase(TIM3, &TIM_TimeBaseStructure);

    TIM_TimeBaseStructure.Period = 73;
    TIM_InitTimeBase(TIM4, &TIM_TimeBaseStructure);

    /* Master Configuration in Toggle Mode */
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_TOGGLE;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = 64;
    TIM_OCInitStructure.OcPolarity  = TIM_OC_POLARITY_HIGH;

    TIM_InitOc1(TIM1, &TIM_OCInitStructure);

    /* TIM1 Input Capture Configuration */
    TIM_ICInitStructure.Channel     = TIM_CH_2;
    TIM_ICInitStructure.IcPolarity  = TIM_IC_POLARITY_RISING;
    TIM_ICInitStructure.IcSelection = TIM_IC_SELECTION_DIRECTTI;
    TIM_ICInitStructure.IcPrescaler = TIM_IC_PSC_DIV1;
    TIM_ICInitStructure.IcFilter    = 0;

    TIM_ICInit(TIM1, &TIM_ICInitStructure);

    /* TIM1 Input trigger configuration: External Trigger connected to TI2 */
    TIM_SelectInputTrig(TIM1, TIM_TRIG_SEL_TI2FP2);
    TIM_SelectSlaveMode(TIM1, TIM_SLAVE_MODE_GATED);

    /* Select the Master Slave Mode */
    TIM_SelectMasterSlaveMode(TIM1, TIM_MASTER_SLAVE_MODE_ENABLE);

    /* Master Mode selection: TIM1 */
    TIM_SelectOutputTrig(TIM1, TIM_TRGO_SRC_ENABLE);

    /* Slaves Configuration: Toggle Mode */
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_TOGGLE;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;

    TIM_InitOc1(TIM3, &TIM_OCInitStructure);

    TIM_InitOc1(TIM4, &TIM_OCInitStructure);

    /* Slave Mode selection: TIM3 */
    TIM_SelectInputTrig(TIM3, TIM_TRIG_SEL_IN_TR0);
    TIM_SelectSlaveMode(TIM3, TIM_SLAVE_MODE_GATED);

    /* Select the Master Slave Mode */
    TIM_SelectMasterSlaveMode(TIM3, TIM_MASTER_SLAVE_MODE_ENABLE);

    /* Master Mode selection: TIM3 */
    TIM_SelectOutputTrig(TIM3, TIM_TRGO_SRC_ENABLE);

    /* Slave Mode selection: TIM4 */
    TIM_SelectInputTrig(TIM4, TIM_TRIG_SEL_IN_TR2);
    TIM_SelectSlaveMode(TIM4, TIM_SLAVE_MODE_GATED);

    /* TIM1 Main Output Enable */
    TIM_EnableCtrlPwmOutputs(TIM1, ENABLE);

    /* TIM enable counter */
    TIM_Enable(TIM1, ENABLE);
    TIM_Enable(TIM3, ENABLE);
    TIM_Enable(TIM4, ENABLE);
        
    while (1)
    {
    }
}

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* TIM1, TIM3 and TIM4 clock enable */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3 | RCC_APB1_PERIPH_TIM4, ENABLE);

    /* TIM1, GPIOA, GPIOE, GPIOC and GPIOB clocks enable */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_TIM1 | RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_GPIOE
                                | RCC_APB2_PERIPH_GPIOC | RCC_APB2_PERIPH_AFIO,
                            ENABLE);
}

/**
 * @brief  Configure the GPIO Pins.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* GPIOA Configuration: PA.08(TIM1 CH1) and PA.06(TIM3 CH1) as alternate function push-pull */
    GPIO_InitStructure.Pin        = GPIO_PIN_8 | GPIO_PIN_6;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    /* GPIOB Configuration: PB.06(TIM4 CH1) as alternate function push-pull */
    GPIO_InitStructure.Pin = GPIO_PIN_6;

    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    /* GPIOA Configuration: PA.09(TIM1 CH2) */
    GPIO_InitStructure.Pin       = GPIO_PIN_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
}

#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param file pointer to the source file name
 * @param line assert_param error line source number
 */
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    while (1)
    {
    }
}

#endif

/**
 * @}
 */

/**
 * @}
 */

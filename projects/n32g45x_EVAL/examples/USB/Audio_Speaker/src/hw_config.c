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
 * @file hw_config.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "n32g45x_tim.h"

#define TIM2ARRValue            4500    /* 16000KHz = 72MHz / 4500 */

/**
 * @brief   Configures USB Clock input (48MHz).
 */
void Set_USBClock(void)
{
    /* Select USBCLK source */
    RCC_ConfigUsbClk(RCC_USBCLK_SRC_PLLCLK_DIV3);
    /* Enable the USB clock */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_USB, ENABLE);
}

/**
 * @brief  Power-off system clocks and power while entering suspend mode.
 */
void Enter_LowPowerMode(void)
{
    /* Set the device state to suspend */
    bDeviceState = SUSPENDED;
}

/**
 * @brief Restores system clocks and power while exiting suspend mode.
 */
void Leave_LowPowerMode(void)
{
    USB_DeviceMess* pInfo = &Device_Info;

    /* Set the device state to the correct state */
    if (pInfo->CurrentConfiguration != 0)
    {
        /* Device configured */
        bDeviceState = CONFIGURED;
    }
    else
    {
        bDeviceState = ATTACHED;
    }
}

/**
 * @brief  Configures the USB interrupts.
 */
void USB_Interrupts_Config(void)
{
    NVIC_InitType NVIC_InitStructure;
    EXTI_InitType EXTI_InitStructure;

    /* 2 bit for pre-emption priority, 2 bits for subpriority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Enable the USB interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
        
    /* Enable and configure the priority of the USB_HP IRQ Channel*/
    NVIC_InitStructure.NVIC_IRQChannel                   = USB_HP_CAN1_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable the USB Wake-up interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = USBWakeUp_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure the EXTI line 18 connected internally to the USB IP */
    EXTI_ClrITPendBit(EXTI_LINE18);
    EXTI_InitStructure.EXTI_Line    = EXTI_LINE18;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);
        
    /* Audio Components Interrupt configuration */
    Audio_Config();
}

/**
 * @brief Configures the TIM2 interrupts
 */
void Audio_Config(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Enable the TIM5 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief Configure and enable the timer
 */
void Speaker_Config(void)
{
    GPIO_InitType GPIO_InitStructure;
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    OCInitType TIM_OCInitStructure;
    
    /* Enable GPIOA, TIM2 & TIM3 clock */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM2 | RCC_APB1_PERIPH_TIM3, ENABLE);
    
    /* Configure PB.01 as alternate function (TIM3_OC4) */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.Pin = GPIO_PIN_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    /* TIM3 configuration */
    TIM_TimeBaseStructure.Prescaler = 0x00;   /* TIM3CLK = 72 MHz */
    TIM_TimeBaseStructure.Period    = 0xFF;   /* PWM frequency : 281.250KHz*/
    TIM_TimeBaseStructure.ClkDiv    = 0x0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_InitTimeBase(TIM3, &TIM_TimeBaseStructure);

    /* TIM3's Channel4 in PWM1 mode */
    TIM_OCInitStructure.OcMode = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.Pulse = 0x7F;  /* Duty cycle: 50%*/
    TIM_OCInitStructure.OcPolarity      = TIM_OC_POLARITY_HIGH;  /* set high polarity */
    TIM_OCInitStructure.OcNPolarity     = TIM_OCN_POLARITY_HIGH;  
    TIM_OCInitStructure.OutputState     = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.OutputNState    = TIM_OUTPUT_NSTATE_DISABLE;
    TIM_OCInitStructure.OcIdleState     = TIM_OC_IDLE_STATE_RESET;
    TIM_OCInitStructure.OcNIdleState    = TIM_OCN_IDLE_STATE_RESET;
    TIM_InitOc4(TIM3, &TIM_OCInitStructure);
    TIM_ConfigOc4Preload(TIM3, TIM_OC_PRE_LOAD_ENABLE);

    /* TIM2 configuration */
    TIM_TimeBaseStructure.Period        = TIM2ARRValue;
    TIM_TimeBaseStructure.Prescaler     = 0x00;    /* TIM2CLK = 72 MHz */
    TIM_TimeBaseStructure.ClkDiv        = 0x0;
    TIM_TimeBaseStructure.CntMode       = TIM_CNT_MODE_UP;
    TIM_InitTimeBase(TIM2, &TIM_TimeBaseStructure);
    /* Output Compare Inactive Mode configuration: Channel1 */
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_TIMING;
    TIM_OCInitStructure.Pulse       = 0x0;
    TIM_InitOc1(TIM2, &TIM_OCInitStructure);
    TIM_ConfigOc1Preload(TIM2, TIM_OC_PRE_LOAD_DISABLE);

    /* Start TIM3 */
    TIM_Enable(TIM3, ENABLE);

    /* Start TIM2 */
    TIM_Enable(TIM2, ENABLE);

    /* Enable TIM2 update interrupt */
    TIM_ConfigInt(TIM2, TIM_INT_UPDATE, ENABLE);
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

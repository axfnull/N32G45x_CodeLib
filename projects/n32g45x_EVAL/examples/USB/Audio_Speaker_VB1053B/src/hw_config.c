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
#include "vs10xx.h"
#include "delay.h"
#include "recorder.h"

ErrorStatus HSEStartUpStatus;
uint32_t ADC_ConvertedValueX     = 0;
uint32_t ADC_ConvertedValueX_1   = 0;
__IO uint16_t ADC1ConvertedValue = 0, ADC1ConvertedVoltage = 0, calibration_value = 0;

ErrorStatus HSEStartUpStatus;
EXTI_InitType EXTI_InitStructure;

#define TIM2ARRValue    1633

/**
 * @brief   Configures Main system clocks & power.
 */
void Set_System(void)
{
    GPIO_InitType GPIO_InitStructure;
    EXTI_InitType EXTI_InitStructure;
 
    /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_n32g45x.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_n32g45x.c file
     */ 

    /* Enable GPIOA, TIM1 & TIM2 clock */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_TIM1, ENABLE);
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM2, ENABLE);

    /* Configure USB pull-up */
    GPIO_InitStructure.Pin = USB_DISCONNECT_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitPeripheral(USB_DISCONNECT_PORT, &GPIO_InitStructure);

    /* Configure the EXTI line 18 connected internally to the USB IP */
    EXTI_ClrITPendBit(EXTI_LINE18);
    EXTI_InitStructure.EXTI_Line = EXTI_LINE18;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);
}

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

    /* Audio Components Interrupt configuration */
    Audio_Config();
}

/**
 * @brief Configures the USB interrupts
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
 * @brief Initializes the GPIO pins used by the codec application.
 */
void I2S_GPIO_Config(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Enable GPIOA, GPIOC and AFIO clock */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOC | RCC_APB2_PERIPH_AFIO, ENABLE);
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_SPI3, ENABLE);
    GPIO_ConfigPinRemap(GPIO_RMP3_SPI3, ENABLE);

    /* I2S3 SD, CK and WS pins configuration */
    GPIO_InitStructure.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    /* I2S2 MCK pin configuration */
    GPIO_InitStructure.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);


    /* I2C1 SCL PB6 and SDA PB7 pins configuration */
    GPIO_InitStructure.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : I2S_Config
* Description    : Configure the I2S Peripheral.
* Input          :  - Standard: I2S_Standard_Phillips, I2S_Standard_MSB or I2S_Standard_LSB
*                   - MCLKOutput: I2S_MCLKOutput_Enable or I2S_MCLKOutput_Disable
*                   - AudioFreq: I2S_AudioFreq_8K, I2S_AudioFreq_16K, I2S_AudioFreq_22K,
*                                I2S_AudioFreq_44K or I2S_AudioFreq_48K
* Output         : None
* Return         : None
*******************************************************************************/
void I2S_Config(uint16_t Standard, uint16_t MCLKOutput, uint16_t AudioFreq)
{
    I2S_InitType I2S_InitStructure;

    /* Enable I2S3 APB1 clock */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_SPI3, ENABLE);

    /* Deinitialize SPI3 peripheral */
    SPI_I2S_DeInit(SPI3);

    /* I2S2 peripheral configuration */
    I2S_InitStructure.I2sMode = I2S_MODE_MASTER_TX;

    /* I2S peripheral configuration */
    I2S_InitStructure.Standard       = Standard;
    I2S_InitStructure.DataFormat     = I2S_DATA_FMT_16BITS;
    I2S_InitStructure.MCLKEnable     = MCLKOutput;
    I2S_InitStructure.AudioFrequency = AudioFreq;
    I2S_InitStructure.CLKPOL         = I2S_CLKPOL_LOW;

    I2S_Init(SPI3, &I2S_InitStructure);
    
    /* Disable the I2S3 TxE interrupt */
    SPI_I2S_EnableInt(SPI3, SPI_I2S_INT_TE, DISABLE);

    /* Enable the I2S3 */
    I2S_Enable(SPI3, ENABLE);
}

/**
 * @brief Configure and enable the timer
 */
void Speaker_Config(void)
{
    /* INIT VS1053 */
    VS_Init();
    /* sine wave test */
    VS_Sine_Test();

    recoder_play(); 
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

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
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

ErrorStatus HSEStartUpStatus;
uint32_t ADC_ConvertedValueX     = 0;
uint32_t ADC_ConvertedValueX_1   = 0;
__IO uint16_t ADC1ConvertedValue = 0, ADC1ConvertedVoltage = 0, calibration_value = 0;

/* Extern variables ----------------------------------------------------------*/
extern __IO uint8_t PrevXferComplete;

GPIO_Module* BUTTON_PORT[BUTTONn]  =  {WAKEUP_BUTTON_GPIO_PORT, TAMPER_BUTTON_GPIO_PORT, 
                                      KEY_BUTTON_GPIO_PORT, RIGHT_BUTTON_GPIO_PORT,
                                      LEFT_BUTTON_GPIO_PORT, UP_BUTTON_GPIO_PORT,
                                      DOWN_BUTTON_GPIO_PORT, SEL_BUTTON_GPIO_PORT}; 

const uint16_t BUTTON_PIN[BUTTONn] = {WAKEUP_BUTTON_PIN, TAMPER_BUTTON_PIN, 
                                      KEY_BUTTON_PIN, RIGHT_BUTTON_PIN,
                                      LEFT_BUTTON_PIN, UP_BUTTON_PIN,
                                      DOWN_BUTTON_PIN, SEL_BUTTON_PIN};

const uint32_t BUTTON_CLK[BUTTONn] = {WAKEUP_BUTTON_GPIO_CLK, TAMPER_BUTTON_GPIO_CLK,
                                      KEY_BUTTON_GPIO_CLK, RIGHT_BUTTON_GPIO_CLK,
                                      LEFT_BUTTON_GPIO_CLK, UP_BUTTON_GPIO_CLK,
                                      DOWN_BUTTON_GPIO_CLK, SEL_BUTTON_GPIO_CLK};

/**
 * @brief  Configures Main system clocks & power.
 */
void Set_System(void)
{
    GPIO_InitType  GPIO_InitStructure;
    EXTI_InitType EXTI_InitStructure;
    
    /*Enable Joystick GPIOs clock*/
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);

    /*Configure the JoyStick IOs as input floating*/
    GPIO_InitStructure.Pin = RIGHT_BUTTON_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;/*PullUp is mandatory for Joystick pins*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(RIGHT_BUTTON_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = LEFT_BUTTON_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;/*PullUp is mandatory for Joystick pins*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LEFT_BUTTON_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = UP_BUTTON_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;/*PullUp is mandatory for Joystick pins*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(UP_BUTTON_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DOWN_BUTTON_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;/*PullUp is mandatory for Joystick pins*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(DOWN_BUTTON_GPIO_PORT, &GPIO_InitStructure);
  
    /* Configure the EXTI line 18 connected internally to the USB IP */
    EXTI_ClrITPendBit(EXTI_LINE18);
    EXTI_InitStructure.EXTI_Line = EXTI_LINE18; 
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);

}

/**
 * @brief  Configures USB Clock input (48MHz).
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
 * @brief  Restores system clocks and power while exiting suspend mode.
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable the USB Wake-up interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = USBWakeUp_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

///*******************************************************************************
//* Function Name  : USB_Cable_Config.
//* Description    : Software Connection/Disconnection of USB Cable.
//* Input          : Cmd: new state.
//* Output         : None.
//* Return         : None
//*******************************************************************************/
// void USB_Cable_Config (FunctionalState Cmd)
//{
//  if (Cmd != DISABLE)
//  {
//    GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
//  }
//  else
//  {
//    GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
//  }
//}

///*******************************************************************************
//* Function Name  : GPIO_Configuration
//* Description    : Configures the different GPIO ports.
//* Input          : None
//* Output         : None
//* Return         : None
//*******************************************************************************/
// void GPIO_Configuration(void)
//{
//  GPIO_InitType GPIO_InitStructure;
//  RCC_EnableAPB2PeriphClk(RCC_APB2Periph_GPIO_DISCONNECT |
//                         RCC_APB2Periph_GPIO_IOAIN , ENABLE);
//  /* USB_DISCONNECT used as USB pull-up */
//  GPIO_InitStructure.Pin = USB_DISCONNECT_PIN;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//  GPIO_InitPeripheral(USB_DISCONNECT, &GPIO_InitStructure);
//  /* Configure Potentiometer IO as analog input */
//  GPIO_InitStructure.Pin = GPIO_IOAIN_PIN;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//  GPIO_InitPeripheral(GPIO_IOAIN, &GPIO_InitStructure);
//}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter can be one of following parameters:    
  *     @arg BUTTON_WAKEUP: Wakeup Push Button
  *     @arg BUTTON_TAMPER: Tamper Push Button  
  *     @arg BUTTON_KEY: Key Push Button 
  *     @arg BUTTON_RIGHT: Joystick Right Push Button 
  *     @arg BUTTON_LEFT: Joystick Left Push Button 
  *     @arg BUTTON_UP: Joystick Up Push Button 
  *     @arg BUTTON_DOWN: Joystick Down Push Button
  *     @arg BUTTON_SEL: Joystick Sel Push Button    
  * @retval The Button GPIO pin value.
  */
uint32_t NATIONS_EVAL_PBGetState(Button_TypeDef Button)
{
    return GPIO_ReadInputDataBit(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}


/**
 * @brief  Decodes the Joystick direction.
 *         that failed. If expr is true, it returns no value.
 * @return The direction value
 */
uint8_t JoyState(void)
{
   /* "right" key is pressed */
     if (!NATIONS_EVAL_PBGetState(Button_RIGHT))
     {
         return JOY_RIGHT;
     }
   /* "left" key is pressed */
     if (!NATIONS_EVAL_PBGetState(Button_LEFT))
     {
         return JOY_LEFT;
     }
   /* "up" key is pressed */
     if (!NATIONS_EVAL_PBGetState(Button_UP))    
     {
         return JOY_UP;
     }
   /* "down" key is pressed */
     if (!NATIONS_EVAL_PBGetState(Button_DOWN))
     {
         return JOY_DOWN;
     }
   /* No key is pressed */
     else
     {
         return 0;
     } 
}

/**
 * @brief   prepares buffer to be sent containing Joystick event infos.
* @param   Keys: keys received from terminal.
 */
void Joystick_Send(uint8_t Keys)
{
    uint8_t Mouse_Buffer[4] = {0, 0, 0, 0};
    int8_t X = 0, Y = 0;

    switch (Keys)
    {
        case JOY_LEFT:
          X -= CURSOR_STEP;
          break;
        case JOY_RIGHT:
          X += CURSOR_STEP;
          break;
        case JOY_UP:
          Y -= CURSOR_STEP;
          break;
        case JOY_DOWN:
          Y += CURSOR_STEP;
          break;
        default:
          return;
    }
    /* prepare buffer to send */
    Mouse_Buffer[1] = X;
    Mouse_Buffer[2] = Y;

    /* Reset the control token to inform upper layer that a transfer is ongoing */
    PrevXferComplete = 0;

    /* Copy mouse position info in ENDP1 Tx Packet Memory Area*/
    USB_SilWrite(EP1_IN, Mouse_Buffer, 4);

    /* Enable endpoint for transmission */
    USB_SetEpTxValid(ENDP1);

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

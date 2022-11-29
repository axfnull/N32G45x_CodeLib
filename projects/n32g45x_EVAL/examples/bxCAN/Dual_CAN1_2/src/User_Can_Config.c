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
 * @file User_Can_Config.c
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */


#include <stdio.h>
#include "n32g45x.h"
#include "n32g45x_can.h"
#include "User_Can_Config.h"
#include "log.h"

CanTxMessage CAN1_TxMessage;
CanTxMessage CAN2_TxMessage;
CanRxMessage CAN1_RxMessage;
CanRxMessage CAN2_RxMessage;

/**
 * @brief  Configures the NVIC for CAN.
 */
void NVIC_Config(void)
{
    NVIC_InitType NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel                   = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel                   = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  Configures CAN1 and CAN2.GPIOs
 */
void CAN_GPIO_Config(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStruct(&GPIO_InitStructure);
    /* Configures CAN1 and CAN2 IOs */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO | RCC_APB2_PERIPH_GPIOD | RCC_APB2_PERIPH_GPIOB, ENABLE);
    /* Configure CAN1 RX pin */
    GPIO_InitStructure.Pin       = GPIO_PIN_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitPeripheral(GPIOD, &GPIO_InitStructure);
    /* Configure CAN1 TX pin */
    GPIO_InitStructure.Pin        = GPIO_PIN_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOD, &GPIO_InitStructure);
    /* Configure CAN2 RX pin */
    GPIO_InitStructure.Pin       = GPIO_PIN_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
    /* Configure CAN2 TX pin */
    GPIO_InitStructure.Pin        = GPIO_PIN_13;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
    /* Remap CAN1 and CAN2 GPIOs */
    GPIO_ConfigPinRemap(GPIO_RMP1_CAN1, ENABLE);
}


/**
 * @brief  Configures CAN Filer.
 */
void CAN1_Filter_Init(void)
{
    CAN_FilterInitType CAN_FilterInitStructure;
    /* CAN filter init */
    CAN_FilterInitStructure.Filter_Num            = CAN_FILTERNUM0;
    CAN_FilterInitStructure.Filter_Mode           = CAN_Filter_IdMaskMode;
    CAN_FilterInitStructure.Filter_Scale          = CAN_Filter_32bitScale;
    CAN_FilterInitStructure.Filter_HighId         = CAN_FILTER_EXTID_H(0x400);
    CAN_FilterInitStructure.Filter_LowId          = CAN_FILTER_EXTID_L(0x400);
    CAN_FilterInitStructure.FilterMask_HighId     = CAN_STD_ID_H_MASK_DONT_CARE;
    CAN_FilterInitStructure.FilterMask_LowId      = CAN_STD_ID_L_MASK_DONT_CARE;
    CAN_FilterInitStructure.Filter_FIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.Filter_Act            = ENABLE;
    CAN1_InitFilter(&CAN_FilterInitStructure);
    CAN_INTConfig(CAN1, CAN_INT_FMP0, ENABLE);
}


/**
 * @brief  Configures CAN Filer.
 */
void CAN2_Filter_Init(void)
{
    CAN_FilterInitType CAN_FilterInitStructure;
    /* CAN filter init */
    CAN_FilterInitStructure.Filter_Num            = CAN_FILTERNUM0;
    CAN_FilterInitStructure.Filter_Mode           = CAN_Filter_IdMaskMode;
    CAN_FilterInitStructure.Filter_Scale          = CAN_Filter_32bitScale;
    CAN_FilterInitStructure.Filter_HighId         = CAN_FILTER_EXTID_H(0x400);
    CAN_FilterInitStructure.Filter_LowId          = CAN_FILTER_EXTID_L(0x400);
    CAN_FilterInitStructure.FilterMask_HighId     = CAN_STD_ID_H_MASK_DONT_CARE;
    CAN_FilterInitStructure.FilterMask_LowId      = CAN_STD_ID_L_MASK_DONT_CARE;
    CAN_FilterInitStructure.Filter_FIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.Filter_Act            = ENABLE;
    CAN2_InitFilter(&CAN_FilterInitStructure);
    CAN_INTConfig(CAN2, CAN_INT_FMP0, ENABLE);
}

/**
 * @brief  Configures CAN1 and CAN2.
 * @param CAN_BaudRate 10Kbit/s ~ 1Mbit/s
 */
void CAN_Config(void)
{
    CAN_InitType CAN_InitStructure;
    /* Configure CAN1 and CAN2 */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_CAN1 | RCC_APB1_PERIPH_CAN2, ENABLE);
    /* CAN1 and CAN2 register init */
    CAN_DeInit(CAN1);
    CAN_DeInit(CAN2);
    /* Struct init*/
    CAN_InitStruct(&CAN_InitStructure);
    /* CAN1 and CAN2  cell init */
    CAN_InitStructure.TTCM          = DISABLE;
    CAN_InitStructure.ABOM          = DISABLE;
    CAN_InitStructure.AWKUM         = DISABLE;
    CAN_InitStructure.NART          = DISABLE;
    CAN_InitStructure.RFLM          = DISABLE;
    CAN_InitStructure.TXFP          = ENABLE;
    CAN_InitStructure.OperatingMode = CAN_Normal_Mode;
    CAN_InitStructure.RSJW          = CAN_BIT_RSJW;
    CAN_InitStructure.TBS1          = CAN_BIT_BS1;
    CAN_InitStructure.TBS2          = CAN_BIT_BS2;
    CAN_InitStructure.BaudRatePrescaler = CAN_BAUDRATEPRESCALER;
    /*Initializes the CAN1  and CAN2 */
    CAN_Init(CAN1, &CAN_InitStructure);
    CAN_Init(CAN2, &CAN_InitStructure);
    CAN1_Filter_Init();
    CAN2_Filter_Init();
    /* IT Configuration for CAN1 */
    CAN_INTConfig(CAN1, CAN_INT_FMP0, ENABLE);
    /* IT Configuration for CAN2 */
    CAN_INTConfig(CAN2, CAN_INT_FMP0, ENABLE);
}

/**
 * @brief  CAN Transmit Message.
 * @param  CAN
 * @param  TxMessage CAN_TxMessage
 * @return The number of the mailbox that is used for transmission or CAN_TxSTS_NoMailBox if there is no empty mailbox.
 */
uint8_t CANTxMessage(CAN_Module* CANx,
                     CanTxMessage* TxMessage)
{
    return CAN_TransmitMessage(CANx, TxMessage);
}

/**
 * @brief  Check Can Receive Message.
 * @param  RxMessage CAN_TxMessage
 * @return FAILED/PASSED
 */
uint8_t Check_CAN2RxMessage(CanRxMessage* RxMessage)
{
    if ((RxMessage->IDE != CAN_ID_STD) || (RxMessage->StdId != 0x0400) )  return FAILED;
    if ( RxMessage->RTR != CAN_RTRQ_DATA) return FAILED;
    if ( RxMessage->DLC != CAN_TXDLC_8)   return FAILED;
    if ((RxMessage->Data[0] != 0x01) || (RxMessage->Data[1] != 0x02) || \
            (RxMessage->Data[2] != 0x03) || (RxMessage->Data[3] != 0x04) || \
            (RxMessage->Data[4] != 0x05) || (RxMessage->Data[5] != 0x06) || \
            (RxMessage->Data[6] != 0x07) || (RxMessage->Data[7] != 0x08) )  return FAILED;
    return PASSED;
}

/**
 * @brief  Check Can Receive Message.
 * @param  RxMessage CAN_TxMessage
 * @return FAILED/PASSED
 */
uint8_t Check_CAN1RxMessage(CanRxMessage* RxMessage)
{
    if ((RxMessage->IDE != CAN_ID_STD) || (RxMessage->StdId != 0x0400) )  return FAILED;
    if ( RxMessage->RTR != CAN_RTRQ_DATA) return FAILED;
    if ( RxMessage->DLC != CAN_TXDLC_8)   return FAILED;
    if ((RxMessage->Data[0] != 0x02) || (RxMessage->Data[1] != 0x02) || \
            (RxMessage->Data[2] != 0x03) || (RxMessage->Data[3] != 0x04) || \
            (RxMessage->Data[4] != 0x05) || (RxMessage->Data[5] != 0x06) || \
            (RxMessage->Data[6] != 0x07) || (RxMessage->Data[7] != 0x08) )  return FAILED;
    return PASSED;
}

/**
 * @brief  Can Transmit Message Data Initialize
 */
void CAN_Tx_Message_Init(void)
{
    /* CAN1 transmit message */
    /* Init Transmit frame*/
    CAN1_TxMessage.StdId    = 0x0400; /* Standard ID or Extended ID(MSBs) */
    CAN1_TxMessage.IDE     = CAN_ID_STD;   /* CAN_ID_STD / CAN_ID_EXT */
    CAN1_TxMessage.RTR     = CAN_RTRQ_DATA;   /* CAN_RTRQ_DATA / CAN_RTRQ_REMOTE */
    CAN1_TxMessage.DLC     = CAN_TXDLC_8;   /* 0 to 8 */
    CAN1_TxMessage.Data[0] = 0x01;CAN1_TxMessage.Data[1] = 0x02;
    CAN1_TxMessage.Data[2] = 0x03;CAN1_TxMessage.Data[3] = 0x04;
    CAN1_TxMessage.Data[4] = 0x05;CAN1_TxMessage.Data[5] = 0x06;
    CAN1_TxMessage.Data[6] = 0x07;CAN1_TxMessage.Data[7] = 0x08;
    
    /* CAN2 transmit message */
    /* Init Transmit frame*/
    CAN2_TxMessage.StdId    = 0x0400; /* Standard ID or Extended ID(MSBs) */
    CAN2_TxMessage.IDE     = CAN_ID_STD;   /* CAN_ID_STD / CAN_ID_EXT */
    CAN2_TxMessage.RTR     = CAN_RTRQ_DATA;   /* CAN_RTRQ_DATA / CAN_RTRQ_REMOTE */
    CAN2_TxMessage.DLC     = CAN_TXDLC_8;   /* 0 to 8 */
    CAN2_TxMessage.Data[0] = 0x02;CAN2_TxMessage.Data[1] = 0x02;
    CAN2_TxMessage.Data[2] = 0x03;CAN2_TxMessage.Data[3] = 0x04;
    CAN2_TxMessage.Data[4] = 0x05;CAN2_TxMessage.Data[5] = 0x06;
    CAN2_TxMessage.Data[6] = 0x07;CAN2_TxMessage.Data[7] = 0x08;
}




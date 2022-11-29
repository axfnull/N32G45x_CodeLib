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
 * @file User_Can_Config.h
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */


#ifndef __USER_CAN_CONFIG_H__
#define __USER_CAN_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif


#define  CAN_BAUDRATE_1M             1
#define  CAN_BAUDRATE_500K           2
#define  CAN_BAUDRATE_250K           3
#define  CAN_BAUDRATE_125K           4
#define  CAN_BAUDRATE                CAN_BAUDRATE_500K

#if(CAN_BAUDRATE==CAN_BAUDRATE_1M)
#define   CAN_BIT_RSJW               CAN_RSJW_1tq
#define   CAN_BIT_BS1                CAN_TBS1_5tq
#define   CAN_BIT_BS2                CAN_TBS2_3tq
#define   CAN_BAUDRATEPRESCALER      4
#elif(CAN_BAUDRATE==CAN_BAUDRATE_500K)
#define   CAN_BIT_RSJW               CAN_RSJW_1tq
#define   CAN_BIT_BS1                CAN_TBS1_14tq
#define   CAN_BIT_BS2                CAN_TBS2_3tq
#define   CAN_BAUDRATEPRESCALER      4
#elif(CAN_BAUDRATE==CAN_BAUDRATE_250K)
#define   CAN_BIT_RSJW               CAN_RSJW_1tq
#define   CAN_BIT_BS1                CAN_TBS1_14tq
#define   CAN_BIT_BS2                CAN_TBS2_3tq
#define   CAN_BAUDRATEPRESCALER      8
#elif(CAN_BAUDRATE==CAN_BAUDRATE_125K)
#define   CAN_BIT_RSJW               CAN_RSJW_2tq
#define   CAN_BIT_BS1                CAN_TBS1_13tq
#define   CAN_BIT_BS2                CAN_TBS2_4tq
#define   CAN_BAUDRATEPRESCALER      16
#endif

#define  CAN_TX_MESSAGE_BUFF_SIZE    2
#define  CAN_RX_MESSAGE_BUFF_SIZE    10

#define  CAN_TXDLC_8                 ((uint8_t)8)
#define  CAN_FILTERNUM0              ((uint8_t)0)
#define  CAN_TX1_SID                 0x0400
#define  CAN_TX2_EID                 0x12345678
#define  CAN_TX_MESSAGE_BUFF_SIZE    2
#define  CAN_RX_MESSAGE_BUFF_SIZE    10

#define  CAN_FILTER_STDID(STDID)     ((STDID&0x7FF)<<5)
#define  CAN_FILTER_EXTID_H(EXTID)   ((uint16_t)(((EXTID>>18)<<5)|((EXTID&0x2FFFF)>>13)))
#define  CAN_FILTER_EXTID_L(EXTID)   ((uint16_t)((EXTID&0x2FFFF)<<3))

#define  CAN_STD_ID_H_MASK_CARE       0xFFE0
#define  CAN_STD_ID_L_MASK_CARE       0x0000
#define  CAN_STD_ID_H_MASK_DONT_CARE  0x0000
#define  CAN_STD_ID_L_MASK_DONT_CARE  0x0000

#define  CAN_EXT_ID_H_MASK_CARE       0xFFFF
#define  CAN_EXT_ID_L_MASK_CARE       0xFFF8
#define  CAN_EXT_ID_H_MASK_DONT_CARE  0x0000
#define  CAN_EXT_ID_L_MASK_DONT_CARE  0x0000

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} Status;

extern uint8_t CAN_Tx_Index;
extern uint8_t CAN_RxMessage_Write_Cursor;
extern uint8_t CAN_RxMessage_Read_Cursor;
extern CanRxMessage CAN_RxMessage[CAN_RX_MESSAGE_BUFF_SIZE];
extern CanTxMessage CAN_TxMessages[CAN_TX_MESSAGE_BUFF_SIZE];

extern void CAN_Tx_Process(void);
extern void CAN_GPIO_Config(void);
extern void CAN_Config(void);
extern void Tx_Frame_Message_Init(void);
extern void CAN_Recieve_Process(void);


#ifdef __cplusplus
}
#endif


#endif/*__USER_CAN_CONFIG_H__*/


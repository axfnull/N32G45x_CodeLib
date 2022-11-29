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
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include <stdio.h>
#include "n32g45x.h"
#include "main.h"
#include "n32g45x_can.h"
#include "User_Can_Config.h"
#include "log.h"
#include "User_Systick_Config.h"

/**
 *  Dual_CAN1_2
 */

/**
 * @brief  Main program.
 */
int main(void)
{
    uint8_t i=0;
    uint8_t TransmitMailbox = 0;
    uint16_t Time_out       = 0xFFFF;
    uint32_t CAN_Transmit_Invert_Time;
    /* USART Init */
    log_init();
    log_info("\r\n Dual CAN1/2 \r\n");
    /* Configures the NVIC for CAN1_RX0/CAN2_RX0 */
    NVIC_Config();
    /* Configures CAN1 and CAN2.IOs */
    CAN_GPIO_Config();
    /* Configures CAN1/CAN2 */
    CAN_Config();
    CAN_Tx_Message_Init();
    SysTick_Config(SystemCoreClock / 1000);
    while (1)
    {
       if(User_Time_Read(CAN_Transmit_Invert_Time)>=TIME_1S)
       {
          User_Time_Set(&CAN_Transmit_Invert_Time);
          TransmitMailbox = CANTxMessage(CAN1,&CAN1_TxMessage);  
          while ((CAN_TransmitSTS(CAN1, TransmitMailbox) != CANTXSTSOK) && (Time_out != 0)) Time_out--;
          Time_out = 0xFFFF;
          while ((CAN_PendingMessage(CAN1, CAN_FIFO0) < 1) && (Time_out != 0)) Time_out--;
          if (FAILED  == Check_CAN2RxMessage(&CAN2_RxMessage)) 
          {
             log_info("\r\n CAN1 transmit message to CAN2, Check CAN2 receive message Failed\r\n");
          }
          else
          {
             log_info("\r\n CAN1 transmit message to CAN2, Check CAN2 receive message Passed\r\n");
          }
          for(i=0;i<8;i++)CAN2_RxMessage.Data[i]=0;
          TransmitMailbox = CANTxMessage(CAN2,&CAN2_TxMessage);  
          while ((CAN_TransmitSTS(CAN2, TransmitMailbox) != CANTXSTSOK) && (Time_out != 0)) Time_out--;
          Time_out = 0xFFFF;
          while ((CAN_PendingMessage(CAN2, CAN_FIFO0) < 1) && (Time_out != 0)) Time_out--;
          if (FAILED  == Check_CAN1RxMessage(&CAN1_RxMessage)) 
          {
             log_info("\r\n CAN2 transmit message to CAN1, Check CAN1 receive message Failed\r\n");
          }
          else
          {
             log_info("\r\n CAN2 transmit message to CAN1, Check CAN1 receive message Passed\r\n");
          }
          for(i=0;i<8;i++)CAN1_RxMessage.Data[i]=0;
       }
    }
}


/**
 * @}
 */

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
*          line: assert_param error line source number
 * @return None
 */
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {}
}

/**
 * @}
 */
#endif

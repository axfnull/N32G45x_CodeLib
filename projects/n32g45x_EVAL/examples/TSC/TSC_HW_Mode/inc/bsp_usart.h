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
 * @file bsp_usart.h
 * @author Nations
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#ifdef __cplusplus
extern "C" {
#endif
    
#include "main.h"


// USART2
#define DEBUG_USARTx                (USART1)
#define DEBUG_USART_CLK             (RCC_APB2_PERIPH_USART1)
#define DEBUG_USART_APBxClkCmd      RCC_EnableAPB2PeriphClk
#define DEBUG_USART_BAUDRATE        (115200)

// Macro definition of USART GPIO pin
#define DEBUG_USART_GPIO_CLK        (RCC_APB2_PERIPH_GPIOA)
#define DEBUG_USART_GPIO_APBxClkCmd RCC_EnableAPB2PeriphClk

#define DEBUG_USART_GPIO_REMAP      (0)

#define DEBUG_USART_TX_GPIO_PORT GPIOA
#define DEBUG_USART_TX_GPIO_PIN  GPIO_PIN_9
#define DEBUG_USART_RX_GPIO_PORT GPIOA
#define DEBUG_USART_RX_GPIO_PIN  GPIO_PIN_10

#define DEBUG_USART_IRQ        USART1_IRQn
//#define DEBUG_USART_IRQHandler USART1_IRQHandler

void Debug_USART_Config(void);
void Usart_SendByte(USART_Module* pUSARTx, uint8_t ch);
void Usart_SendString(USART_Module* pUSARTx, char* str);
void Usart_SendHalfWord(USART_Module* pUSARTx, uint16_t ch);
void Debug_USART_DeInit(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_USART_H__ */

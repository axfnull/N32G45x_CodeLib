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
 * @file main.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __MAIN_H__
#define __MAIN_H__

#include "n32g45x.h"
#include "n32g45x_conf.h"

#define I2S_MASTER        SPI3
#define I2S_MASTER_CLK    RCC_APB1_PERIPH_SPI3
#define I2S_MASTER_PIN_CK GPIO_PIN_3
#define I2S_MASTER_PIN_SD GPIO_PIN_5
#define I2S_MASTER_PIN_WS GPIO_PIN_15

#define I2S_MASTER_DMA            DMA2
#define I2S_MASTER_DMA_CLK        RCC_AHB_PERIPH_DMA2
#define I2S_MASTER_Rx_DMA_Channel DMA2_CH1
#define I2S_MASTER_Rx_DMA_FLAG    DMA2_FLAG_TC1
#define I2S_MASTER_Tx_DMA_Channel DMA2_CH2
#define I2S_MASTER_Tx_DMA_FLAG    DMA2_FLAG_TC2

#define I2S_SLAVE        SPI2
#define I2S_SLAVE_CLK    RCC_APB1_PERIPH_SPI2
#define I2S_SLAVE_PIN_CK GPIO_PIN_12
#define I2S_SLAVE_PIN_SD GPIO_PIN_13
#define I2S_SLAVE_PIN_WS GPIO_PIN_14

#define I2S_SLAVE_DMA            DMA1
#define I2S_SLAVE_DMA_CLK        RCC_AHB_PERIPH_DMA1
#define I2S_SLAVE_Rx_DMA_Channel DMA1_CH4
#define I2S_SLAVE_Rx_DMA_FLAG    DMA1_FLAG_TC4
#define I2S_SLAVE_Tx_DMA_Channel DMA1_CH5
#define I2S_SLAVE_Tx_DMA_FLAG    DMA1_FLAG_TC5

#define I2S_SLAVE_DR_Base  0x4000380C
#define I2S_MASTER_DR_Base 0x40003C0C

/* Uncomment the line corresponding to the desired System clock (SYSCLK)
   frequency (after reset the HSI is used as SYSCLK source) */
//#define SYSCLK_HSI
//#define SYSCLK_HSE
#define SYSCLK_PLLHSI_FREQ_24MHz
//#define SYSCLK_PLLHSI_FREQ_36MHz
//#define SYSCLK_PLLHSI_FREQ_48MHz
//#define SYSCLK_PLLHSI_FREQ_56MHz
//#define SYSCLK_PLLHSI_FREQ_72MHz
//#define SYSCLK_PLLHSI_FREQ_96MHz
//#define SYSCLK_PLLHSE_FREQ_24MHz
//#define SYSCLK_PLLHSE_FREQ_36MHz
//#define SYSCLK_PLLHSE_FREQ_48MHz
//#define SYSCLK_PLLHSE_FREQ_56MHz
//#define SYSCLK_PLLHSE_FREQ_72MHz
//#define SYSCLK_PLLHSE_FREQ_96MHz
//#define SYSCLK_PLLHSE_FREQ_144MHz

#endif /* __MAIN_H__ */

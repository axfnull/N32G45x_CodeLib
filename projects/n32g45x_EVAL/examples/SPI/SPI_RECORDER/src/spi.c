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
 * @file spi.c
 * @author Nations 
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "spi.h"

/**
 * @brief  Initializes SPI3
 */
void SPI3_Init(void)
{
    GPIO_InitType GPIO_InitStructure;
    SPI_InitType  SPI_InitStructure;
     
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOC | RCC_APB2_PERIPH_AFIO, ENABLE);
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_SPI3, ENABLE);
    GPIO_ConfigPinRemap(GPIO_RMP3_SPI3, ENABLE);
    
    GPIO_InitStructure.Pin        = SPI_MASTER_PIN_SCK;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitPeripheral(SPI_MASTER_SCK_GPIO, &GPIO_InitStructure);
    
    /*!< Configure sFLASH_SPI pins: MOSI */
    GPIO_InitStructure.Pin = SPI_MASTER_PIN_MOSI;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitPeripheral(SPI_MASTER_MOSI_GPIO, &GPIO_InitStructure);

    /*!< Configure sFLASH_SPI pins: MISO */
    GPIO_InitStructure.Pin       = SPI_MASTER_PIN_MISO;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitPeripheral(SPI_MASTER_MISO_GPIO, &GPIO_InitStructure);
    
    SPI_InitStructure.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
    SPI_InitStructure.SpiMode       = SPI_MODE_MASTER;
    SPI_InitStructure.DataLen       = SPI_DATA_SIZE_8BITS;
    SPI_InitStructure.CLKPOL        = SPI_CLKPOL_HIGH;
    SPI_InitStructure.CLKPHA        = SPI_CLKPHA_SECOND_EDGE;
    SPI_InitStructure.NSS           = SPI_NSS_SOFT;
    SPI_InitStructure.BaudRatePres  = SPI_BR_PRESCALER_128;
    SPI_InitStructure.FirstBit      = SPI_FB_MSB;
    SPI_InitStructure.CRCPoly       = 7;
    SPI_Init(SPI3, &SPI_InitStructure);

    SPI_Enable(SPI3, ENABLE);
    SPI3_ReadWriteByte(0xff);//start transmit
}

/**
 * @brief  set spi speed
 * @param  SPI_BaudRatePrescaler
 * @return None
 */

void SPI3_SetSpeed(u8 SpeedSet)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
    SPI3->CTRL1&=0XFFC7; 
    SPI3->CTRL1|=SpeedSet;
    SPI_Enable(SPI3,ENABLE); 
}

/**
 * @brief  Sends a byte through the SPI interface and return the byte received
 *         from the SPI bus.
 * @param byte byte to send.
 * @return The value of the received byte.
 */
uint8_t SPI3_ReadWriteByte(uint8_t TxData)
{
    uint8_t retry = 0;
    /*!< Loop while DAT register in not emplty */
    while (SPI_I2S_GetStatus(SPI3, SPI_I2S_TE_FLAG) == RESET)
    {
        retry++;
        if(retry>200)
            return 0;
    }   
    /*!< Send byte through the SPI1 peripheral */
    SPI_I2S_TransmitData(SPI3, TxData);
    retry=0;
        
    /*!< Wait to receive a byte */
    while (SPI_I2S_GetStatus(SPI3, SPI_I2S_RNE_FLAG) == RESET)
    {
        retry++;
        if(retry>200)
            return 0;
    }   
    /*!< Return the byte read from the SPI bus */
    return SPI_I2S_ReceiveData(SPI3);           
}

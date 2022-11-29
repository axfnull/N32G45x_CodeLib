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
#include "n32g45x.h"
#include "main.h"

/** @addtogroup N32G45X_StdPeriph_Examples
 * @{
 */

/** @addtogroup I2S_DMA
 * @{
 */

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

#define BufferSize 32

DMA_InitType DMA_InitStructure;
GPIO_InitType GPIO_InitStructure;
I2S_InitType I2S_InitStructure;

// uint8_t I2S_MASTER_Buffer_Tx[BufferSize] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
//                                            0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C,
//                                            0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12,
//                                            0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
//                                            0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E,
//                                            0x1F, 0x20};
const uint16_t I2S_MASTER_Buffer_Tx[BufferSize] = {0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E, 0x0F10,
                                                   0x1112, 0x1314, 0x1516, 0x1718, 0x191A, 0x1B1C, 0x1D1E, 0x1F20,
                                                   0x2122, 0x2324, 0x2526, 0x2728, 0x292A, 0x2B2C, 0x2D2E, 0x2F30,
                                                   0x3132, 0x3334, 0x3536, 0x3738, 0x393A, 0x3B3C, 0x3D3E, 0x3F40};
uint16_t I2S_SLAVE_Buffer_Rx[BufferSize];
volatile TestStatus TransferStatus = FAILED;
ErrorStatus HSEStartUpStatus;

void RCC_Configuration(void);
void GPIO_Configuration(void);
TestStatus Buffercmp(uint16_t* pBuffer1, uint16_t* pBuffer2, uint16_t BufferLength);

/**
 * @brief  Main program.
 */
int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured,
        this is done through SystemInit() function which is called from startup
        file (startup_n32g45x.s) before to branch to application main.
        To reconfigure the default setting of SystemInit() function, refer to
        system_n32g45x.c file
      */

    /* System clocks configuration ---------------------------------------------*/
    RCC_Configuration();

    /* GPIO configuration ------------------------------------------------------*/
    GPIO_Configuration();

    /* Deinitializes the SPI2 and SPI3 peripheral registers --------------------*/
    SPI_I2S_DeInit(I2S_SLAVE);
    SPI_I2S_DeInit(I2S_MASTER);

    /* I2S_SLAVE_Rx_DMA_Channel configuration ---------------------------------------------*/
    DMA_DeInit(I2S_SLAVE_Rx_DMA_Channel);
    DMA_InitStructure.PeriphAddr     = (uint32_t)I2S_SLAVE_DR_Base;
    DMA_InitStructure.MemAddr        = (uint32_t)I2S_SLAVE_Buffer_Rx;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.BufSize        = BufferSize;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_HALFWORD;
    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.CircularMode   = DMA_MODE_NORMAL;
    DMA_InitStructure.Priority       = DMA_PRIORITY_VERY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;
    DMA_Init(I2S_SLAVE_Rx_DMA_Channel, &DMA_InitStructure);

    /* I2S_MASTER_Tx_DMA_Channel configuration ---------------------------------------------*/
    DMA_DeInit(I2S_MASTER_Tx_DMA_Channel);
    DMA_InitStructure.PeriphAddr = (uint32_t)I2S_MASTER_DR_Base;
    DMA_InitStructure.MemAddr    = (uint32_t)I2S_MASTER_Buffer_Tx;
    DMA_InitStructure.Direction  = DMA_DIR_PERIPH_DST;
    DMA_Init(I2S_MASTER_Tx_DMA_Channel, &DMA_InitStructure);

    /* I2S peripheral configuration */
    I2S_InitStructure.Standard       = I2S_STD_PHILLIPS;
    I2S_InitStructure.DataFormat     = I2S_DATA_FMT_16BITS_EXTENDED;
    I2S_InitStructure.MCLKEnable     = I2S_MCLK_DISABLE;
    I2S_InitStructure.AudioFrequency = I2S_AUDIO_FREQ_48K;
    I2S_InitStructure.CLKPOL         = I2S_CLKPOL_LOW;

    /* I2S3 Master Transmitter to I2S2 Slave Receiver communication ------------*/
    /* I2S3 configuration */
    I2S_InitStructure.I2sMode = I2S_MODE_MASTER_TX;
    I2S_Init(I2S_MASTER, &I2S_InitStructure);

    /* I2S2 configuration */
    I2S_InitStructure.I2sMode = I2S_MODE_SlAVE_RX;
    I2S_Init(I2S_SLAVE, &I2S_InitStructure);

    /* Enable I2S_SLAVE Rx request */
    SPI_I2S_EnableDma(I2S_SLAVE, SPI_I2S_DMA_RX, ENABLE);
    /* Enable I2S_MASTER Tx request */
    SPI_I2S_EnableDma(I2S_MASTER, SPI_I2S_DMA_TX, ENABLE);

    /* Enable the I2S2 */
    I2S_Enable(I2S_SLAVE, ENABLE);

    /* Enable the I2S3 */
    I2S_Enable(I2S_MASTER, ENABLE);

    /* Enable DMA2 Channel2 */
    DMA_EnableChannel(I2S_MASTER_Tx_DMA_Channel, ENABLE);
    /* Enable DMA1 Channel4 */
    DMA_EnableChannel(I2S_SLAVE_Rx_DMA_Channel, ENABLE);

    /* Wait for DMA2 channel2 transfer complete */
    while (!DMA_GetFlagStatus(I2S_MASTER_Tx_DMA_FLAG, DMA2))
        ;
    while (!DMA_GetFlagStatus(I2S_SLAVE_Rx_DMA_FLAG, DMA1))
        ;

    /* Check the correctness of written data */
    TransferStatus = Buffercmp(I2S_SLAVE_Buffer_Rx, (uint16_t*)I2S_MASTER_Buffer_Tx, BufferSize);
    /* TransferStatus = PASSED, if the transmitted and received data
       are equal */
    /* TransferStatus = FAILED, if the transmitted and received data
       are different */

    while (1)
    {
    }
}

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();

    /* Enable HSE */
    RCC_ConfigHse(RCC_HSE_ENABLE);

    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitHseStable();

    if (HSEStartUpStatus == SUCCESS)
    {
        /* Enable Prefetch Buffer */
        FLASH_PrefetchBufSet(FLASH_PrefetchBuf_EN);

        /* Flash 2 wait state */
        FLASH_SetLatency(FLASH_LATENCY_2);

        /* HCLK = SYSCLK */
        RCC_ConfigHclk(RCC_SYSCLK_DIV1);

        /* PCLK2 = HCLK */
        RCC_ConfigPclk2(RCC_HCLK_DIV1);

        /* PCLK1 = HCLK/2 */
        RCC_ConfigPclk1(RCC_HCLK_DIV2);

        /* PLLCLK = 8MHz * 9 = 72 MHz */
        RCC_ConfigPll(RCC_PLL_SRC_HSE_DIV1, RCC_PLL_MUL_9);

        /* Enable PLL */
        RCC_EnablePll(ENABLE);

        /* Wait till PLL is ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRD) == RESET)
        {
        }

        /* Select PLL as system clock source */
        RCC_ConfigSysclk(RCC_SYSCLK_SRC_PLLCLK);

        /* Wait till PLL is used as system clock source */
        while (RCC_GetSysclkSrc() != 0x08)
        {
        }
    }

    /* Enable peripheral clocks --------------------------------------------------*/
    /* Enable I2S_SLAVE DMA clock */
    RCC_EnableAHBPeriphClk(I2S_SLAVE_DMA_CLK, ENABLE);
    RCC_EnableAHBPeriphClk(I2S_MASTER_DMA_CLK, ENABLE);

    /* GPIOA, GPIOB and AFIO clocks enable */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);

    /* SPI2 and SPI3 clocks enable */
    RCC_EnableAPB1PeriphClk(I2S_SLAVE_CLK | I2S_MASTER_CLK, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Disable the JTAG interface and enable the SWJ interface
        This operation is not necessary for Connectivity Line devices since
        SPI3 I/Os can be remapped on other GPIO pins */
    GPIO_ConfigPinRemap(GPIO_RMP_SW_JTAG_SW_ENABLE, ENABLE);

    /* Configure SPI2 pins: CK, WS and SD ---------------------------------*/
    GPIO_InitStructure.Pin        = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    /* Configure SPI3 pins: CK and SD ------------------------------------*/
    GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_5;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    /* Configure SPI3 pins: WS -------------------------------------------*/
    GPIO_InitStructure.Pin = GPIO_PIN_15;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
}

/**
 * @brief  Compares two buffers.
 * @param  pBuffer1, pBuffer2: buffers to be compared.
 * @param BufferLength buffer's length
 * @return PASSED: pBuffer1 identical to pBuffer2
 *         FAILED: pBuffer1 differs from pBuffer2
 */
TestStatus Buffercmp(uint16_t* pBuffer1, uint16_t* pBuffer2, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer1 != *pBuffer2)
        {
            return FAILED;
        }

        pBuffer1++;
        pBuffer2++;
    }

    return PASSED;
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

    /* Infinite loop */
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

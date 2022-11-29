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
 * @file dvp_demo.c
 * @author Nations
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "dvp_demo.h"
#include "gc0308.h"

#define DVP_InitCamera()        (GC0308_Init())
#define DVP_REMAP_MODE          (GPIO_RMP1_DVP)

uint32_t DVP_Image[DVP_IMAGE_BUFF_SIZE] = {0};

/**
 * @brief   Init the port used for DVP interface.
 * @param   nUs specifies the delay time based on 1us
 * @retval: None
 */
static void DVP_InitPort(void)
{
    GPIO_InitType GPIO_InitStruct;

    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    if (DVP_REMAP_MODE == 0) /* No remap */
    {
        /*  PA1:HSYNC   PA2:VSYNC   PA3:PCLK
            PA4:D0      PA5:D1      PA6:D2      PA7:D3
            PC4:D4      PC5:D5      PB0:D6      PB1:D7*/
        RCC_EnableAPB2PeriphClk( RCC_APB2_PERIPH_GPIOA
                                |RCC_APB2_PERIPH_GPIOB
                                |RCC_APB2_PERIPH_GPIOC, 
                                 ENABLE);

        /*  PORTA */
        GPIO_InitStruct.Pin = GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  
                            | GPIO_PIN_4  | GPIO_PIN_5  | GPIO_PIN_6  
                            | GPIO_PIN_7;
        GPIO_InitPeripheral(GPIOA, &GPIO_InitStruct);

        /*  PORTB */
        GPIO_InitStruct.Pin = GPIO_PIN_0  | GPIO_PIN_1;
        GPIO_InitPeripheral(GPIOB, &GPIO_InitStruct);

        /*  PORTC */
        GPIO_InitStruct.Pin = GPIO_PIN_4  | GPIO_PIN_5;
        GPIO_InitPeripheral(GPIOC, &GPIO_InitStruct);
    }
    else if (DVP_REMAP_MODE == GPIO_RMP1_DVP)    /* Remap==1 */
    {
        /*  PE2:HSYNC   PE3:VSYNC   PE4:PCLK
            PE5:D0      PE6:D1      PC0:D2      PB2:D3
            PF12:D4     PF13:D5     PF14:D6     PF15:D7 */
        RCC_EnableAPB2PeriphClk( RCC_APB2_PERIPH_GPIOE
                                |RCC_APB2_PERIPH_GPIOB
                                |RCC_APB2_PERIPH_GPIOC
                                |RCC_APB2_PERIPH_GPIOF
                                |RCC_APB2_PERIPH_AFIO, 
                                 ENABLE);

        /* Pin remap */
        GPIO_ConfigPinRemap(GPIO_RMP1_DVP, ENABLE);

        /* PORTE */
        GPIO_InitStruct.Pin = GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_4  
                            | GPIO_PIN_5  | GPIO_PIN_6;
        GPIO_InitPeripheral(GPIOE, &GPIO_InitStruct);

        /* PORTC */
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitPeripheral(GPIOC, &GPIO_InitStruct);

        /* PORTB */
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitPeripheral(GPIOB, &GPIO_InitStruct);

        /* PORTF */
        GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14  
                            | GPIO_PIN_15;
        GPIO_InitPeripheral(GPIOF, &GPIO_InitStruct);
    }
    else if (DVP_REMAP_MODE == GPIO_RMP3_DVP)        /* Remap==3 */
    {
        /*  PE2:HSYNC   PE3:VSYNC   PE4:PCLK
            PE5:D0      PE6:D1      PC0:D2      PB2:D3
            PB10:D4     PB11:D5     PF14:D6     PF15:D7 */
        RCC_EnableAPB2PeriphClk( RCC_APB2_PERIPH_GPIOE
                                |RCC_APB2_PERIPH_GPIOB
                                |RCC_APB2_PERIPH_GPIOC
                                |RCC_APB2_PERIPH_GPIOF
                                |RCC_APB2_PERIPH_AFIO, 
                                 ENABLE);

        /* Pin remap */
        GPIO_ConfigPinRemap(GPIO_RMP3_DVP, ENABLE);

        /* PORTE */
        GPIO_InitStruct.Pin = GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_4  
                            | GPIO_PIN_5  | GPIO_PIN_6;
        GPIO_InitPeripheral(GPIOE, &GPIO_InitStruct);

        /* PORTC */
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitPeripheral(GPIOC, &GPIO_InitStruct);

        /* PORTB */
        GPIO_InitStruct.Pin = GPIO_PIN_2  | GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitPeripheral(GPIOB, &GPIO_InitStruct);

        /* PORTF */
        GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitPeripheral(GPIOF, &GPIO_InitStruct);
    }
    else
    {
        while (1)
            ;
    }
}

/**
 * @brief   Init the DMA used for DVP interface.
 * @param   None
 * @retval: None
 */
static void DVPDemo_InitDMA(void)
{
    DMA_InitType DMA_InitStructure;

    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_DMA2, ENABLE);

    DMA_DeInit(DMA2_CH8);

    DMA_InitStructure.MemAddr        = (uint32_t)&DVP_Image;
    DMA_InitStructure.PeriphAddr     = (uint32_t) & (DVP->FIFO);
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.BufSize        = DVP_IMAGE_BUFF_SIZE;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_WORD;
    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_Word;
    DMA_InitStructure.CircularMode   = DMA_MODE_CIRCULAR;
    DMA_InitStructure.Priority       = DMA_PRIORITY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;

    DMA_Init(DMA2_CH8, &DMA_InitStructure);
    DMA_EnableChannel(DMA2_CH8, ENABLE);
}

/**
 * @brief   Init the registers for DVP interface module.
 * @param   None
 * @retval: None
 */
static void DVP_InitInterface(void)
{
    DVP_InitType DVP_InitStruct;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_DVP, ENABLE);

    DVP_ResetReg();

    DVP_InitStruct.FifoWatermark    = DVP_WATER_MARK_1;
    DVP_InitStruct.LineCapture      = DVP_LINE_CAPTURE_ALL;
    DVP_InitStruct.ByteCapture      = DVP_BYTE_CAPTURE_ALL;
    DVP_InitStruct.DataInvert       = DVP_DATA_NOTINVERT;
    DVP_InitStruct.PixelClkPolarity = DVP_PIXEL_POLARITY_FALLING;
    DVP_InitStruct.VsyncPolarity    = DVP_VSYNC_POLARITY_LOW;
    DVP_InitStruct.HsyncPolarity    = DVP_HSYNC_POLARITY_HIGH;
    DVP_InitStruct.CaptureMode      = DVP_CAPTURE_MODE_SINGLE;
    DVP_InitStruct.RowStart         = 0;
    DVP_InitStruct.ColumnStart      = 0;
    DVP_InitStruct.ImageHeight      = DVP_IMAGE_HEIGHT;
    DVP_InitStruct.ImageWidth       = DVP_IMAGE_SIZE/DVP_IMAGE_HEIGHT;
    DVP_Init(&DVP_InitStruct);
}

/**
 * @brief   Init DVP demo.
 * @param   None
 * @retval: None
 */
void DVPDemo_Init(void)
{
    DVPDemo_InitDMA();
    DVP_InitPort();
    DVP_InitInterface();
    DVP_InitCamera();
    DVP_ConfigDma(ENABLE);
    DelayUs(1000);
}

/**
 * @brief   Capture a picture by DVP interface.
 * @param   None
 * @retval: None
 */
void DVPDemo_Capture(void)
{
    DVP_ResetFifo();    /*Clear FIFO*/
    __DVP_ClrFlag(DVP_CLEAR_FLAG_FME);  /*Clear frame end flag*/
//    __DVP_EnableDMA();  /*Enable DVP DMA */
    __DVP_StartCapture();     /*Enable DVP capture*/

    while(__DVP_FlagIsNotSet(DVP_FLAG_FME));    /*Wait frame capture done.*/

    __DVP_StopCapture();    /*Diable DVP capture*/
//    __DVP_DisableDMA(); /*Diable DVP DMA*/
}



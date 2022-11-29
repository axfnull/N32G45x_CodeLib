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
#include "main.h"

/** @addtogroup ADC_ADC1_TEMP
 * @{
 */

ADC_InitType ADC_InitStructure;
ADC_InitTypeEx ADC_InitStructureEx;
DMA_InitType DMA_InitStructure;
__IO uint16_t ADCConvertedValue;


void RCC_Configuration(void);
void Delay(__IO uint32_t nCount);
void USART1_Config(void);


#define Vc0        0    //X
#define Tc1        1.5f
/*V30 is the voltage value at 30 degree Celsius by factory default*/ 
uint16_t  V30 = 0; 
uint16_t  ADvalue = 0;
uint32_t  temp = 0;

#ifdef  DEFINEFLOAT   //Float
__IO float TempValue;
/*xx mv per degree Celsius  by datasheet define*/
#define AVG_SLOPE  0.0041f
/**
 * @brief  Cal temp use float result.
 */
float TempCal(uint16_t TempAdVal)
{
    float Temperate;
    /* Get the temperature inside the chip */
    Temperate=((V30+Vc0-TempAdVal)*3.3/4095)/AVG_SLOPE+30.0f-Tc1;
    return Temperate;
}
#else
__IO int16_t TempValue;
/*xx mv per degree Celsius  by datasheet define*/
#define AVG_SLOPE  41
/**
 * @brief  Cal temp use integer result.
 */
 
int16_t TempCal(uint16_t TempAdVal)
{
    int16_t Temperate;
    /* Get the temperature inside the chip */
    Temperate=(int16_t)((((V30+Vc0-TempAdVal)*33*10000)/(4095*10))/AVG_SLOPE+30-Tc1);
    return Temperate;
}
#endif


/**
 * @brief   Main program
 */
int main(void)
{
    /* System clocks configuration ---------------------------------------------*/
    RCC_Configuration();

    /* DMA1 channel1 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_CH1);
    DMA_InitStructure.PeriphAddr     = (uint32_t)&ADC1->DAT;
    DMA_InitStructure.MemAddr        = (uint32_t)&ADCConvertedValue;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.BufSize        = 1;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_DISABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_HALFWORD;
    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.CircularMode   = DMA_MODE_CIRCULAR;
    DMA_InitStructure.Priority       = DMA_PRIORITY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;
    DMA_Init(DMA1_CH1, &DMA_InitStructure);
    /* Enable DMA1 channel1 */
    DMA_EnableChannel(DMA1_CH1, ENABLE);

    /* ADC1 configuration ------------------------------------------------------*/
    ADC_InitStructure.WorkMode       = ADC_WORKMODE_INDEPENDENT;
    ADC_InitStructure.MultiChEn      = ENABLE;
    ADC_InitStructure.ContinueConvEn = ENABLE;
    ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIGCONV_NONE;
    ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber      = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    /* ADC1 temp sensor enable */
    ADC_EnableTempSensorVrefint(ENABLE);  
        
    ADC_InitStructureEx.VbatMinitEn = ENABLE;
    ADC_InitStructureEx.DeepPowerModEn = DISABLE;
    ADC_InitStructureEx.JendcIntEn = DISABLE;
    ADC_InitStructureEx.EndcIntEn = DISABLE;
    ADC_InitStructureEx.ClkMode = ADC_CTRL3_CKMOD_AHB;
    ADC_InitStructureEx.CalAtuoLoadEn = DISABLE;
    ADC_InitStructureEx.DifModCal = false;
    ADC_InitStructureEx.ResBit = ADC_CTRL3_RES_12BIT;
    ADC_InitStructureEx.SampSecondStyle = false;
    ADC_InitEx(ADC1, &ADC_InitStructureEx);
    /* ADC1 regular configuration */
    ADC_ConfigRegularChannel(ADC1, ADC_CH_16, 1, ADC_SAMP_TIME_239CYCLES5);//ADC_CH_16：测温度，ADC_CH_18：测内部1.2V基准
    /* ADC1 temp sensor enable */
    ADC_EnableTempSensorVrefint(ENABLE);  
    /* Enable ADC1 DMA */
    ADC_EnableDMA(ADC1, ENABLE);

    /* Enable ADC1 */
    ADC_Enable(ADC1, ENABLE);
    /*Check ADC Ready*/
    while(ADC_GetFlagStatusNew(ADC1,ADC_FLAG_RDY) == RESET)
        ;
    /* Start ADC1 calibration */
    ADC_StartCalibration(ADC1);
    /* Check the end of ADC1 calibration */
    while (ADC_GetCalibrationStatus(ADC1))
        ;
        
    /* Start ADC1 Software Conversion */
    ADC_EnableSoftwareStartConv(ADC1, ENABLE);

    /* Config Uart1 as Temperature output */
    USART1_Config();
    
    V30 = *(__IO uint32_t*)((uint32_t)0x1FFFF7D0);
    while (1)
    {
        /* */
        TempValue = TempCal(ADCConvertedValue);
        #ifdef DEFINEFLOAT
        printf("\r\n Temperature = %.3f C\r\n",TempValue); 
        #else
        printf("\r\n Temperature = %d C\r\n",TempValue); 
        #endif
        
        Delay(500);
    }
}

/*
 * @brief  uart configure as 115200 8-N-1
 */
void USART1_Config(void)
{
    GPIO_InitType GPIO_InitStructure;
    USART_InitType USART_InitStructure;
    
    /* configure USART1  clock*/
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_USART1 | RCC_APB2_PERIPH_GPIOA, ENABLE);
    
    /* USART1 GPIO config */
    /* Configure USART1 Tx (PA.09) as alternate function push-pull */
    GPIO_InitStructure.Pin = GPIO_PIN_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);    
    /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.Pin = GPIO_PIN_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
      
    /* USART1 work mode*/
    USART_InitStructure.BaudRate = 115200;
    USART_InitStructure.WordLength = USART_WL_8B;
    USART_InitStructure.StopBits = USART_STPB_1;
    USART_InitStructure.Parity = USART_PE_NO ;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;
    USART_Init(USART1, &USART_InitStructure); 
    USART_Enable(USART1, ENABLE);
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE* f)
{
    USART_SendData(USART1, (uint8_t)ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXDE) == RESET)
        ;

    return (ch);
}

/**
 * @brief  Delay funciton.
 */
void Delay(__IO uint32_t nCount)
{
    for(; nCount != 0; nCount--);
} 
/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* Enable peripheral clocks ------------------------------------------------*/
    /* Enable DMA1 and DMA2 clocks */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_DMA1 | RCC_AHB_PERIPH_DMA2, ENABLE);

    /* Enable GPIOC clocks */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC, ENABLE);
    /* Enable ADC1 clocks */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC1,ENABLE);

    /* RCC_ADCHCLK_DIV16*/
    ADC_ConfigClk(ADC_CTRL3_CKMOD_AHB,RCC_ADCHCLK_DIV16);
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

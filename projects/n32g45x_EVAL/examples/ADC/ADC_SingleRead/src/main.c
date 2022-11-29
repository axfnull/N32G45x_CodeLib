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
#include "main.h"

/** @addtogroup ADC_ADC1_DMA
 * @{
 */

ADC_InitType ADC_InitStructure;
DMA_InitType DMA_InitStructure;
__IO uint16_t ADC1ConvertedValue[5],ADC2ConvertedValue[5],ADC3ConvertedValue[5],ADC4ConvertedValue[5];

void RCC_Configuration(void);
void GPIO_Configuration(void);
uint16_t ADC_GetData(ADC_Module* ADCx, uint8_t ADC_Channel);
void ADC_Initial(ADC_Module* ADCx);

void ADC_Initial(ADC_Module* ADCx)
{
    /* ADC configuration ------------------------------------------------------*/
    ADC_InitStructure.WorkMode       = ADC_WORKMODE_INDEPENDENT;
    ADC_InitStructure.MultiChEn      = DISABLE;
    ADC_InitStructure.ContinueConvEn = DISABLE;
    ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIGCONV_NONE;
    ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber      = 1;
    ADC_Init(ADCx, &ADC_InitStructure);


    /* Enable ADC */
    ADC_Enable(ADCx, ENABLE);
    /*Check ADC Ready*/
    while(ADC_GetFlagStatusNew(ADCx,ADC_FLAG_RDY) == RESET)
        ;
    /* Start ADC calibration */
    ADC_StartCalibration(ADCx);
    /* Check the end of ADC calibration */
    while (ADC_GetCalibrationStatus(ADCx))
        ;
}
/**
 * @brief   Main program
 */
int main(void)
{
    /* System clocks configuration ---------------------------------------------*/
    RCC_Configuration();

    /* GPIO configuration ------------------------------------------------------*/
    GPIO_Configuration();
    
    ADC_Initial(ADC1);
    ADC_Initial(ADC2);
    ADC_Initial(ADC3);
    ADC_Initial(ADC4);

    while (1)
    {
        ADC1ConvertedValue[0]=ADC_GetData(ADC1,ADC1_Channel_06_PC0);
        ADC1ConvertedValue[1]=ADC_GetData(ADC1,ADC1_Channel_07_PC1);
        
        ADC2ConvertedValue[0]=ADC_GetData(ADC2,ADC2_Channel_08_PC2);
        ADC2ConvertedValue[1]=ADC_GetData(ADC2,ADC2_Channel_09_PC3);
        
        ADC3ConvertedValue[0]=ADC_GetData(ADC3,ADC3_Channel_07_PD10);
        ADC3ConvertedValue[1]=ADC_GetData(ADC3,ADC3_Channel_08_PD11);
        
        ADC4ConvertedValue[0]=ADC_GetData(ADC4,ADC4_Channel_09_PD12);
        ADC4ConvertedValue[1]=ADC_GetData(ADC4,ADC4_Channel_10_PD13);
    }
}

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* Enable peripheral clocks ------------------------------------------------*/

    /* Enable GPIOC clocks */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOF|RCC_APB2_PERIPH_GPIOG|RCC_APB2_PERIPH_GPIOA|RCC_APB2_PERIPH_GPIOB|RCC_APB2_PERIPH_GPIOC|RCC_APB2_PERIPH_GPIOD|RCC_APB2_PERIPH_GPIOE, ENABLE);
    /* Enable ADC1, ADC2, ADC3 and ADC4 clocks */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC1 | RCC_AHB_PERIPH_ADC2 | RCC_AHB_PERIPH_ADC3 | RCC_AHB_PERIPH_ADC4,
                           ENABLE);

    /* RCC_ADCHCLK_DIV16*/
    ADC_ConfigClk(ADC_CTRL3_CKMOD_AHB,RCC_ADCHCLK_DIV16);
    RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8);  //selsect HSE as RCC ADC1M CLK Source		
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;
    
    /* Configure PC0 PC1 as analog input -------------------------*/
    GPIO_InitStructure.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
    /* Configure PC2 PC3 as analog input -------------------------*/
    GPIO_InitStructure.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
    /* Configure PD10 PD11 as analog input -------------------------*/
    GPIO_InitStructure.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitPeripheral(GPIOD, &GPIO_InitStructure);
    /* Configure PD12 PD13 as analog input -------------------------*/
    GPIO_InitStructure.Pin       = GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitPeripheral(GPIOD, &GPIO_InitStructure);
}

uint16_t ADC_GetData(ADC_Module* ADCx, uint8_t ADC_Channel)
{
    uint16_t dat;
    
    ADC_ConfigRegularChannel(ADCx, ADC_Channel, 1, ADC_SAMP_TIME_239CYCLES5);
    /* Start ADC Software Conversion */
    ADC_EnableSoftwareStartConv(ADCx, ENABLE);
    while(ADC_GetFlagStatus(ADCx, ADC_FLAG_ENDC)==0){
    }
    ADC_ClearFlag(ADCx, ADC_FLAG_ENDC);
    ADC_ClearFlag(ADCx, ADC_FLAG_STR);
    dat=ADC_GetDat(ADCx);
    return dat;
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

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

#include "n32g45x.h"
#include "n32g45x_conf.h"
#include "main.h"
#include "log.h"
#include "User_LED_Config.h"


/** @addtogroup N32G45X_StdPeriph_Examples
 * @{
 */

/** @addtogroup
 * @{
 */

__IO uint32_t TimingDelay = 0;
__IO uint32_t LsiFreq     = 40000;

/**
 * @brief  Inserts a delay time.
 * @param nTime specifies the delay time length, in milliseconds.
 */
void Delay(__IO uint32_t nTime)
{
    TimingDelay = nTime;
    while (TimingDelay != 0);
}

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
    log_init();
    log_info("\r\n IWDG demo reset \r\n");
    LEDInit(LED1_PORT,LED1_PIN);
    LEDInit(LED2_PORT,LED2_PIN);
    LEDOff(LED1_PORT,LED1_PIN);
    LEDOff(LED2_PORT,LED2_PIN);
    /* Setup SysTick Timer for 1 msec interrupts  */
    if (SysTick_Config(SystemCoreClock / 1000))
    {
       /* Capture error */
       while (1);
    }
    /* Check if the system has resumed from IWDG reset */
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
    {
       /* IWDGRST flag set */
       /* Turn on LED1 */
       LEDOn(LED1_PORT,LED1_PIN);
       log_info("\r\n reset by IWDG \r\n");
       /* Clear reset flags */
       RCC_ClrFlag();
    }
    else
    {
       /* IWDGRST flag is not set */
       /* Turn off LED1 */
       LEDOff(LED1_PORT,LED1_PIN);
    }
#ifdef LSI_TIM_MEASURE
    /* Enable the LSI OSC */
    RCC_EnableLsi(ENABLE);
    /* Wait till LSI is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRD) == RESET)
    {
    }
    /* TIM Configuration -------------------------------------------------------*/
    TIM5_ConfigForLSI();
    /* Wait until the TIM5 get 3 LSI edges */
    while (CaptureNumber != 3)
    {
    }
    /* Disable TIM5 CC4 Interrupt Request */
    TIM_ConfigInt(TIM5, TIM_INT_CC4, DISABLE);
#endif
    /* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency
       dispersion) */
    /* Enable write access to IWDG_PR and IWDG_RLR registers */
    IWDG_WriteConfig(IWDG_WRITE_ENABLE);
    /* IWDG counter clock: LSI/32 */
    IWDG_SetPrescalerDiv(IWDG_PRESCALER_DIV8);
    /* Set counter reload value to obtain 250ms IWDG TimeOut.
       Counter Reload Value = 250ms/IWDG counter clock period
                            = 250ms / (LSI/32)
                            = 0.25s / (LsiFreq/32)
                            = LsiFreq/(32 * 4)
                            = LsiFreq/128
     */
    log_debug("LsiFreq is: %d\n", LsiFreq);
    IWDG_CntReload(LsiFreq / 128);
    /* Reload IWDG counter */
    IWDG_ReloadKey();
    /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
    IWDG_Enable();
    while (1)
    {
       /* Toggle LED1 */
       LEDBlink(LED1_PORT, LED1_PIN);
       /* Insert 249 ms delay */
       Delay(62);
       /* Reload IWDG counter */
       IWDG_ReloadKey();
    }
}



#ifdef LSI_TIM_MEASURE
/**
 * @brief  Configures TIM5 to measure the LSI oscillator frequency.
 */
void TIM5_ConfigForLSI(void)
{
    NVIC_InitType NVIC_InitStructure;
    TIM_ICInitType TIM_ICInitStructure;
    /* Enable TIM5 clocks */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM5, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
    /* Enable the TIM5 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /* Configure TIM5 prescaler */
    TIM_ConfigPrescaler(TIM5, 0, TIM_PSC_RELOAD_MODE_IMMEDIATE);
    /* Connect internally the TM5_CH4 Input Capture to the LSI clock output */
    GPIO_ConfigPinRemap(GPIO_RMP_TIM5CH4, ENABLE);
    /* TIM5 configuration: Input Capture mode 
       The LSI oscillator is connected to TIM5 CH4
       The Rising edge is used as active edge,
       The TIM5 CCDAT4 is used to compute the frequency value */
    TIM_ICInitStructure.Channel     = TIM_CH_4;
    TIM_ICInitStructure.IcPolarity  = TIM_IC_POLARITY_RISING;
    TIM_ICInitStructure.IcSelection = TIM_IC_SELECTION_DIRECTTI;
    TIM_ICInitStructure.IcPrescaler = TIM_IC_PSC_DIV8;
    TIM_ICInitStructure.IcFilter    = 0;
    TIM_ICInit(TIM5, &TIM_ICInitStructure);
    /* TIM10 Counter Enable */
    TIM_Enable(TIM5, ENABLE);
    /* Reset the flags */
    TIM5->STS = 0;
    /* Enable the CC4 Interrupt Request */
    TIM_ConfigInt(TIM5, TIM_INT_CC4, ENABLE);
}
#endif

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

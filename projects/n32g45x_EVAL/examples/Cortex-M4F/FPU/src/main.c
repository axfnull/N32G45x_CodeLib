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
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "main.h"
#include <stdio.h>
#include "string.h"
#include "julia_fpu.h"

/**
 *  Cortex-M4F FPU
 */

#define DEMO_USART_BAUDRATE ((uint32_t)115200)

#define SCREEN_X_SIZE    ((uint16_t)320)
#define SCREEN_Y_SIZE    ((uint8_t)208)
#define PBAR_X_POS       ((uint16_t)0x100)
#define PBAR_Y_POS_H     ((uint8_t)0)
#define PBAR_Y_POS_L     ((uint8_t)227)
#define PBAR_COLOR       ((uint16_t)0xf79e)
#define ANIMATION_LENGHT ((uint32_t)26)

#define SUBMODE_FPU_USED_MODE     ((uint8_t)0)
#define SUBMODE_FPU_NOT_USED_MODE ((uint8_t)1)

#if (__FPU_USED == 1)
#define SCORE_FPU_MODE    "FPU On"
#define PROGRESS_FPU_MODE "FPU ON"
#else
#define SCORE_FPU_MODE    "FPU Off"
#define PROGRESS_FPU_MODE "FPU OFF"
#endif /* __FPU_USED */

const int16_t animation_zoom[ANIMATION_LENGHT] = {120, 110,  100,  150,  200,  275,  350,  450,  600,
                                                  800, 1000, 1200, 1500, 2000, 1500, 1200, 1000, 800,
                                                  600, 450,  350,  275,  200,  150,  100,  110};
TIM_TimeBaseInitType TIM_TimeBaseStructure;
uint8_t buffer[SCREEN_X_SIZE * SCREEN_Y_SIZE];
uint8_t text[50];
__IO FlagStatus key_pressed;
__IO FlagStatus switch_mode;

/**
 * @brief  Main program.
 */
int main(void)
{
    /* Animation pointer */
    uint8_t animation_pointer;
    /* Benchmark result */
    uint32_t score_fpu;

    /* USART Init */
    USART_Config();
    printf("Cortex-M4F FPU \r\n");

    /* Initialise global variables */
    switch_mode       = RESET;
    animation_pointer = 0;
    /* Initialise local variables */
    score_fpu = 0;

    /* Clear Screen */
    memset(buffer, 0x00, sizeof(buffer));

    InitTIMER();

    /* Start generating the fractal */
    while (1)
    {
        /* Start the timer */
        StartTIMER();

        /* Make the calculation */
        GenerateJulia_fpu(SCREEN_X_SIZE,
                          SCREEN_Y_SIZE,
                          SCREEN_X_SIZE / 2,
                          SCREEN_Y_SIZE / 2,
                          animation_zoom[animation_pointer],
                          buffer);

        /* Get elapsed time */
        score_fpu = StopTIMER();

        printf("%s : %dms\r\n", SCORE_FPU_MODE, score_fpu >> 2);
        break;
    }
}

/**
 * @brief  Initializes Timer for execution time measurement
 */
void InitTIMER(void)
{
    RCC_ClocksType RCC_Clocks;

    /* TIM1 clock enable */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_TIM1, ENABLE);

    RCC_GetClocksFreqValue(&RCC_Clocks);

    /* DeInit TIM1 */
    TIM_DeInit(TIM1);

    /* Time Base configuration */
    TIM_TimeBaseStructure.Prescaler = ((RCC_Clocks.Pclk1Freq / 1000));
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_TimeBaseStructure.Period    = 65535;
    TIM_TimeBaseStructure.ClkDiv    = 0;
    TIM_TimeBaseStructure.RepetCnt  = 0;

    TIM_InitTimeBase(TIM1, &TIM_TimeBaseStructure);
}

/**
 * @brief  Resets timer counter and start counting
 */
void StartTIMER(void)
{
    /* Disable TIM1 counter */
    TIM_Enable(TIM1, DISABLE);

    /* Initialize counter */
    TIM1->CNT = 0x0000;

    /* TIM1 counter enable */
    TIM_Enable(TIM1, ENABLE);
}

/**
 * @brief  Stops timer count and returns counter value
 * @return Counter Value
 */
uint32_t StopTIMER(void)
{
    /* TIM1 counter enable */
    TIM_Enable(TIM1, DISABLE);

    /* Return counter value */
    return (uint32_t)TIM1->CNT;
}

/* Public functions */

/**
 * @brief  USART_Config.
 */
void USART_Config(void)
{
    GPIO_InitType GPIO_InitStructure;
    USART_InitType USART_InitStructure;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_USART1, ENABLE);

    GPIO_InitStructure.Pin        = GPIO_PIN_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Pin       = GPIO_PIN_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.BaudRate            = DEMO_USART_BAUDRATE;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;
    USART_Init(USART1, &USART_InitStructure);

    USART_Enable(USART1, ENABLE);
}

/**
 * @}
 */

/**
 * @brief  Retargets the C library printf function to the USART1.
 * @param
 * @return
 */
int fputc(int ch, FILE* f)
{
    USART_SendData(USART1, (uint8_t)ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXDE) == RESET)
        ;
    return (ch);
}

/*  */
/**
 * @brief  Retargets the C library scanf function to the USART1.
 * @param
 * @return
 */
int fgetc(FILE* f)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXDNE) == RESET)
        ;
    return (int)USART_ReceiveData(USART1);
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

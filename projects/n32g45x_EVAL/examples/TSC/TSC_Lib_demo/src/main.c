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
 * @version v1.0.3
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "main.h"
#include "bsp_usart.h"
#include "bsp_timer.h"
#include "bsp_tsc.h"
#include "bsp_gpio.h"
#include "bsp_it.h"
#include "rtc_hal.h"

uint8_t gTscStop2Data[TSC_ALG_HANDLER_STOP2_DATA_SIZE] = {0};

/**
 * @brief  Set HSI as system clock
 */
void SetSysClockToHSI(void)
{
    RCC_DeInit();

    RCC_EnableHsi(ENABLE);

    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufSet(FLASH_PrefetchBuf_EN);

    /* Flash 0 wait state */
    FLASH_SetLatency(FLASH_LATENCY_0);

    /* ENABLE iCACHE */
    FLASH_iCacheCmd(FLASH_iCache_EN);

    /* HCLK = SYSCLK */
    RCC_ConfigHclk(RCC_SYSCLK_DIV1);

    /* PCLK2 = HCLK */
    RCC_ConfigPclk2(RCC_HCLK_DIV1);

    /* PCLK1 = HCLK */
    RCC_ConfigPclk1(RCC_HCLK_DIV1);

    /* Select HSE as system clock source */
    RCC_ConfigSysclk(RCC_SYSCLK_SRC_HSI);

    /* Wait till PLL is used as system clock source */
    while (RCC_GetSysclkSrc() != 0x00)
    {
    }
}

/**
 * @brief  Set PLL as system clock
 */
void SetSysClockToPLL(uint32_t freq, uint8_t src)
{
    uint32_t pllsrc, pllmul, latency, pclk1div, pclk2div;
    ErrorStatus ClockStatus;

    if(src == SYSCLK_PLLSRC_HSE)
    {
        if (HSE_VALUE != 8000000)
        {
            /* HSE_VALUE == 8000000 is needed in this project! */
            log_error("HSE is not 8MHz!\r\n");
            while (1);
        }
        
        pllsrc = RCC_PLL_SRC_HSE_DIV2;
    }
    else
        pllsrc = RCC_PLL_SRC_HSI_DIV2;
    
    /* RCC system reset */
    RCC_DeInit();

    /* Enable pll source clock */
    if (src == SYSCLK_PLLSRC_HSE)
    {
        /* Enable HSE */
        RCC_ConfigHse(RCC_HSE_ENABLE);

        /* Wait till HSE is ready */
        ClockStatus = RCC_WaitHseStable();

        if (ClockStatus != SUCCESS)
        {
            /* If HSE fails to start-up, the application will have wrong clock
               configuration. User can add here some code to deal with this
               error */
            log_error("HSE error!\r\n");
            
            /* Go to infinite loop */
            while (1);
        }
    }
    else
    {
//        RCC_EnableHsi(ENABLE);
        while(RCC_GetFlagStatus(RCC_FLAG_HSIRD) == RESET);
    }

    /* Config the divider factor*/
    switch (freq)
    {
    case 24000000:
        latency  = FLASH_LATENCY_0;
        pllmul   = RCC_PLL_MUL_6;
        pclk1div = RCC_HCLK_DIV1;
        pclk2div = RCC_HCLK_DIV1;
        break;
    case 36000000:
        latency  = FLASH_LATENCY_1;
        pllmul   = RCC_PLL_MUL_9;
        pclk1div = RCC_HCLK_DIV1;
        pclk2div = RCC_HCLK_DIV1;
        break;
    case 48000000:
        latency  = FLASH_LATENCY_1;
        pllmul   = RCC_PLL_MUL_12;
        pclk1div = RCC_HCLK_DIV2;
        pclk2div = RCC_HCLK_DIV1;
        break;
    case 56000000:
        latency  = FLASH_LATENCY_1;
        pllmul   = RCC_PLL_MUL_14;
        pclk1div = RCC_HCLK_DIV2;
        pclk2div = RCC_HCLK_DIV1;
        break;
    case 72000000:
        latency  = FLASH_LATENCY_2;
        pllmul   = RCC_PLL_MUL_18;
        pclk1div = RCC_HCLK_DIV2;
        pclk2div = RCC_HCLK_DIV1;
        break;
    case 96000000:
        latency  = FLASH_LATENCY_2;
        pllmul   = RCC_PLL_MUL_24;
        pclk1div = RCC_HCLK_DIV4;
        pclk2div = RCC_HCLK_DIV2;
        break;
    case 128000000:
        latency  = FLASH_LATENCY_3;
        pllmul   = RCC_PLL_MUL_32;
        pclk1div = RCC_HCLK_DIV4;
        pclk2div = RCC_HCLK_DIV2;
        break;
    case 144000000:
        /* must use HSE as PLL source */
        latency  = FLASH_LATENCY_4;
        pllsrc   = RCC_PLL_SRC_HSE_DIV1;
        pllmul   = RCC_PLL_MUL_18;
        pclk1div = RCC_HCLK_DIV4;
        pclk2div = RCC_HCLK_DIV2;
        break;
    default:
        while (1)
            ;
    }

    FLASH_SetLatency(latency);
    
    /* HCLK = SYSCLK */
    RCC_ConfigHclk(RCC_SYSCLK_DIV1);

    /* PCLK2 = HCLK */
    RCC_ConfigPclk2(pclk2div);

    /* PCLK1 = HCLK */
    RCC_ConfigPclk1(pclk1div);

    RCC_ConfigPll(pllsrc, pllmul);

    /* Enable PLL */
    RCC_EnablePll(ENABLE);

    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRD) == RESET)
        ;

    /* Select PLL as system clock source */
    RCC_ConfigSysclk(RCC_SYSCLK_SRC_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while (RCC_GetSysclkSrc() != 0x08)
        ;

    SystemCoreClockUpdate();
}

/**
 * @brief  Function after wakeup
 */
static void wakeup_from_stop2(void)
{
    SystemInit();
    SetSysClockToHSI();
    SetSysClockToPLL(128000000, SYSCLK_PLLSRC_HSI);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    tsc_wakeup_init();
    log_init();
    log_debug("system restart...\r\n");

    led_gpio_init();
    TIM6_init();
    
    touch_ctrl_init();
}

/**
 * @brief  Main program.
 */
int main(void)
{
    uint32_t time_cnt = 0;

    SetSysClockToHSI();
    SetSysClockToPLL(128000000, SYSCLK_PLLSRC_HSI);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    log_init();
    time_cnt = 1000000;
    while(time_cnt--);
    log_info("**********TSC Demo used algo lib**********\r\n");
    time_cnt = 0;
    
    RTC_CLKSourceConfig(RTC_CLK_SRC_LSI);
    led_gpio_init();
    Tsc_PoweronInit();
    TIM6_init();
    
    touch_ctrl_init();

  #ifdef TSC_LOWPOWER_DEBUG
        DBG_ConfigPeriph(DBG_SLEEP|DBG_STOP,ENABLE);
  #endif

    while (1)
    {
        //touch
        touch_ctrl();

        if (Flag_1s == 1)
        {
            Flag_1s = 0;
            log_info("@@counter %d\r\n", time_cnt);
            
            if (time_cnt++ >= 10)
            {
                time_cnt = 0;
                log_info("enter stop2\r\n");
                //设置触控低功耗唤醒
                tsc_alg_set_powerdown(0);

                RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
                PWR_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
                
                wakeup_from_stop2();
                log_debug("have touch\r\n");
            }
        }
    }
}


/******************************************************************/

/**
 * @brief Assert failed function by user.
 * @param file The name of the call that failed.
 * @param line The source line number of the call that failed.
 */
#ifdef USE_FULL_ASSERT
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
#if LOG_ENABLE
    if(DebugUartInited)
        log_error("assert failed:%s at %s (line %d)",expr, file, line);
#endif
    while (1)
    {
    }
}
#endif // USE_FULL_ASSERT
/******************************************************************/



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
#include "n32g45x_eth.h"
#include "n32g45x_rcc.h"
#include "n32g45x_gpio.h"
#include "n32g45x_exti.h"
#include "lwip/netif.h"
#include "lwip/api.h"
#include "lwip/sockets.h"
#include "lwipopts.h"
#include "lwip_port.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "misc.h"
#include "log.h"

// #include "app_main.h"

int errno = 0;

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    printf("stack overflow\n");
}

void vApplicationMallocFailedHook(void)
{
    printf("malloc failed\n");
}

void init_system(void)
{
    // SYSCLK must be 100M, otherwise MCO cannot be configured to 50M
    if (SystemCoreClock != 100000000)
    {
        while (1)
            ;
    }

    // Configure MCO 50M
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);

    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStructure.Pin        = GPIO_PIN_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    RCC_ConfigMcoPllClk(RCC_MCO_PLLCLK_DIV2); // MCO 50M
    RCC_ConfigMco(RCC_MCO_PLLCLK);

    for (int i = 0; i < 100000; ++i)
    {
    }

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO | RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_GPIOC
                                | RCC_APB2_PERIPH_GPIOD | RCC_APB2_PERIPH_GPIOE | RCC_APB2_PERIPH_GPIOF
                                | RCC_APB2_PERIPH_GPIOG,
                            ENABLE);

    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ETHMAC, ENABLE);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
}

extern void test_tcp_init(void);
int main(void)
{
    init_system();
    log_init();
    log_info(" reset\n");
    RCC_ClocksType clks;
    RCC_GetClocksFreqValue(&clks);

    log_info("SYSCLK: %d\n", clks.SysclkFreq);
    log_info("HCLK: %d\n", clks.HclkFreq);
    log_info("PCLK1: %d\n", clks.Pclk1Freq);
    log_info("PCLK2: %d\n", clks.Pclk2Freq);

    if (lwip_port_init())
    {
        printf("LWIP Init Falied!\n");
        while (1)
            ;
    }
    log_info("LWIP Init Success!\n");

    test_tcp_init();
    
    log_info("TCP Task Success!\n");

    vTaskStartScheduler();
    while (1)
        ;
}

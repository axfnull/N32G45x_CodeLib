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
 * @file bsp_gpio.c
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "bsp_gpio.h"

/***
 * @brief    Configure the ports of TSC channels for sample.
 */
void tsc_gpio_init(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed   = GPIO_INPUT;

    RCC_EnableAPB2PeriphClk( RCC_APB2_PERIPH_GPIOA
                            |RCC_APB2_PERIPH_GPIOB
                            |RCC_APB2_PERIPH_GPIOC, ENABLE);

    /*  tsc GPIOB port*/
    /*  PB6: TSC_CH20   */
    GPIO_InitStructure.Pin          = GPIO_PIN_6;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    /* tsc GPIOC port */
    /*  PC6: TSC_CH8       PC7: TSC_CH9 */
    GPIO_InitStructure.Pin  = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);


    /*PA9: TSC OUT*/
    GPIO_InitStructure.Pin              = GPIO_PIN_9;
    GPIO_InitStructure.GPIO_Mode        = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed       = GPIO_Speed_2MHz;
    GPIO_InitPeripheral(GPIOA,&GPIO_InitStructure);
}

/***
 * @brief    Configure the ports of led.
 */
void led_gpio_init(void)
{
    GPIO_InitType GPIO_InitStructure;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);

    GPIO_ResetBits(GPIOB,GPIO_PIN_10);
    
    /*PB10*/
    GPIO_InitStructure.Pin          = GPIO_PIN_10;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_2MHz;
    GPIO_InitPeripheral(GPIOB,&GPIO_InitStructure); 
}

/***
* @brief    Toggle the led to blink.
*/
void led3_blink(void)
{
    GPIOB->POD ^= GPIO_PIN_10;
}





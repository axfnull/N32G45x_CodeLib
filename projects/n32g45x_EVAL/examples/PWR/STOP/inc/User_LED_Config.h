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
 * @file User_LED_Config.h
 * @author Nations 
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */


#ifndef __USER_LED_CONFIG_H__
#define __USER_LED_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif


#define LED1_PORT      GPIOA
#define LED2_PORT      GPIOB
#define LED3_PORT      GPIOB

#define LED1_PIN       GPIO_PIN_8
#define LED2_PIN       GPIO_PIN_4
#define LED3_PIN       GPIO_PIN_5

extern void LEDBlink(GPIO_Module* GPIOx,uint16_t Pin);
extern void LEDOn(GPIO_Module* GPIOx,uint16_t Pin);
extern void LEDOff(GPIO_Module* GPIOx,uint16_t Pin);
extern void LEDInit(GPIO_Module* GPIOx, uint16_t Pin);

#ifdef __cplusplus
}
#endif


#endif/*__USER_LED_CONFIG_H__*/

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
 * @file hw_config.h
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "usb_type.h"
#include "n32g45x.h"

#define KEY_A_PIN               GPIO_PIN_4
#define KEY_B_PIN               GPIO_PIN_5
#define KEY_C_PIN               GPIO_PIN_6

#define KEY_PORT                GPIOA

#define LED_CAPSLOCK_PIN        GPIO_PIN_5
#define LED_NUMLOCK_PIN         GPIO_PIN_8

#define LED_CAPSLOCK_PORT       GPIOB
#define LED_NUMLOCK_PORT        GPIOA

#define open_capslock_led()   GPIO_SetBits(LED_CAPSLOCK_PORT, LED_CAPSLOCK_PIN);
#define close_capslock_led()  GPIO_ResetBits(LED_CAPSLOCK_PORT, LED_CAPSLOCK_PIN);

#define open_numlock_led()   GPIO_SetBits(LED_NUMLOCK_PORT, LED_NUMLOCK_PIN);
#define close_numlock_led()  GPIO_ResetBits(LED_NUMLOCK_PORT, LED_NUMLOCK_PIN);

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/

//#define USB_LOW_PWR_MGMT_SUPPORT

void Set_System(void);
void Set_USBClock(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void TimingDelay_Decrement(void);

#endif /*__HW_CONFIG_H__*/

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

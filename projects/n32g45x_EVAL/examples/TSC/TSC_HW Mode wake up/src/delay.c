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
 * @file delay.c
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "delay.h"

static u8 us_base  = 0; // Number of systick clocks required per us.
static u16 ms_base = 0; // Number of systick clocks required per ms.

/**
 * @brief  Initialization delay function.
 */
void delay_init(void)
{
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); // Select external clock,HCLK/8.
    us_base = SystemCoreClock / 8000000;                  // Number of systick clocks required per us.
    ms_base = (u16)us_base * 1000;                        // Number of systick clocks required per ms.
}

/**
 * @brief  Function of delay nus.
 * @param  u32 nus:Delay number of us.
 */
void delay_us(u32 nus)
{
    u32 temp;
    SysTick->LOAD = nus * us_base;            // load the time
    SysTick->VAL  = 0x00;                     // clear the counter
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // start to countdown
    do
    {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16))); // waiting time arrives
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;      // turn off the counter
    SysTick->VAL = 0X00;                            // clear the counter
}
/**
 * @brief  Function of delay nms.Note the range of nms,less than or equal to 0xffffff*8*1000/SYSCLK.While SYSCLK is
 * equal to 72MHz,nms<=1864.
 * @param  u32 nms:Delay number of ms.
 */
void delay_ms(u16 nms)
{
    u32 temp;
    SysTick->LOAD = (u32)nms * ms_base;       // load the time
    SysTick->VAL  = 0x00;                     // clear counter
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // start to countdown
    do
    {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16))); // waiting time arrives
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;      // turn off the counter
    SysTick->VAL = 0X00;                            // clear the counter
}

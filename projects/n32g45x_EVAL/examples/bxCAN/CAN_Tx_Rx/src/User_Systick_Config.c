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
 * @file User_Systick_Config.c
 * @author Nations 
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */


#include "n32g45x.h"
#include "system_n32g45x.h"
#include "User_Systick_Config.h"

uint32_t systick_count;

/**
 * @brief  Systick Config
 */
void Systick_MS_Config(uint32_t sys_freq)
{
   /* Setup SysTick Timer for 1 msec interrupts  SystemCoreClock */
   if (SysTick_Config(sys_freq / 1000))
   {
      /* Capture error */
      while (1);
   }
}

/**
 * @brief  Systick Disable
 */
void Systick_Disable(void)
{
    SysTick->CTRL &= (~SysTick_CTRL_ENABLE_Msk); 
}


/**
 * @brief  Read User Time Since Last Set
 * @param last time.
 */
uint32_t User_Time_Read(uint32_t time)
{
    if(systick_count>=time)
    {
       return systick_count-time;
    }
    return(0xFFFFFFFF-time+systick_count);
}

/**
 * @brief  Set The User Time By Current  Value
 * @param set time.
 */
void User_Time_Set(uint32_t* time)
{
   *time=systick_count;
}

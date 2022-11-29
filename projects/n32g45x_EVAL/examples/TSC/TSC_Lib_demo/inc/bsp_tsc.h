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
 * @file bsp_tsc.h
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __BSP_TSC_H__
#define __BSP_TSC_H__

#ifdef __cplusplus
extern "C" {
#endif
    
#include "main.h"

//#define THRE_DATA2PC_DEBUG  (1)   //触控数据输出到PC宏开关
//#define THRE_TOUCHDATA_DEBUG  (1) //触控按键消息打印宏开关


#define TSC_USED_RAM_SIZE   (460 * 4)   //触控通道使用的RAM空间大小


typedef struct
{
    uint8_t sram[TSC_USED_RAM_SIZE];    //触控通道使用的RAM空间大小
    uint32_t KeyHold[MAX_TSC_HW_CHN];   //触控通道的按下持续时间。单位ms
    uint32_t Status; //触控任务状态

    uint32_t WakeupFlag;         //指示是否正常唤醒，还是干扰唤醒.1：正常唤醒.0:干扰唤醒
    uint32_t WakeupDelay;         //唤醒后的等待时间计数

  #ifdef THRE_TOUCHDATA_DEBUG //监控模式下设定log缓存空间
    uint16_t    LogBuf[260*4];
  #endif
} TSC_KEY_INFO;

extern TSC_KEY_INFO TscKey;

int32_t app_touch_init(void);
int32_t touch_ctrl_init(void);
int32_t touch_ctrl_powerdown(void);
uint32_t touch_wakeup_check(void);
void touch_ctrl(void);
void Tsc_PoweronInit(void);
void tsc_wakeup_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TSC_H__ */


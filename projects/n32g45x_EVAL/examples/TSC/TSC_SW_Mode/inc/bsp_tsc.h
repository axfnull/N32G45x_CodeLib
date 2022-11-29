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
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __BSP_TSC_H__
#define __BSP_TSC_H__

#ifdef __cplusplus
extern "C" {
#endif
    
#include "n32g45x.h"
#include <stdio.h>
#include "n32g45x_tsc.h"

#define SUM_NUM  10     /* The number of samples for each channel during every check period */
#define KEY_NUM  2     /* The number of total used touch keys */

typedef struct
{
    uint32_t EtrCnt;                        // ETR value of this time count by TIMER(TIMx->CNT)
    uint32_t EtrCntPre;                     // ETR value of last time
    uint32_t PulseSample[KEY_NUM][SUM_NUM]; //ETR sample cnt, KEY_NUM:channel SUM_NUM:samples of each channel
    uint32_t EtrSumThreshold[KEY_NUM];      // Threshold of the sum of all samples for each channel.
    uint32_t key_status;                    // Key status for each channel,bitx =1 press =0 release  x=0~(KEY_NUM-1)
    uint32_t key_status_pre;                // Last key status
    uint32_t KeyHoldCnt[KEY_NUM];           // The hold time when key pressed,reserved
    /*for test*/
    uint32_t MaxPressSum[KEY_NUM];
    uint32_t MinPressSum[KEY_NUM];
    uint32_t MaxReleaseSum[KEY_NUM];
    uint32_t MinReleaseSum[KEY_NUM];
} TSC_ETR_KEY_PARA;


void BSP_TSC_SW_Init(void);
void BSP_TSC_KeyCheck(void);
void BSP_TSC_KeyPrint(uint32_t current_key, uint32_t last_key);
void BSP_TSC_TimIrqCheck(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TSC_H__ */


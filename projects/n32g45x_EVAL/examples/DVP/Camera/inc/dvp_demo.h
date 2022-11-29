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
 * @file dvp_demo.h
 * @author Nations
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __DVP_DEMO_H__
#define __DVP_DEMO_H__

#ifdef __cplusplus
extern "C" {
#endif
    
#include "main.h"

#define DVP_IMAGE_FORMAT_GRAY       (1)
#define DVP_IMAGE_FORMAT_RGB565     (2)
#define DVP_IMAGE_FORMAT            (DVP_IMAGE_FORMAT_RGB565)

#define DVP_IMAGE_PIXEL_SIZE        (DVP_IMAGE_FORMAT)

#if (DVP_IMAGE_FORMAT == DVP_IMAGE_FORMAT_GRAY)
    #define DVP_IMAGE_WIDTH             ((uint16_t)320)
    #define DVP_IMAGE_HEIGHT            ((uint16_t)240)
#else
    #define DVP_IMAGE_WIDTH             ((uint16_t)274)
    #define DVP_IMAGE_HEIGHT            ((uint16_t)206)
#endif

#define DVP_IMAGE_SIZE              (DVP_IMAGE_WIDTH * DVP_IMAGE_HEIGHT * DVP_IMAGE_PIXEL_SIZE)
#define DVP_IMAGE_BUFF_SIZE         ((DVP_IMAGE_SIZE+3)/4)

extern RCC_ClocksType clks;
extern uint32_t DVP_Image[DVP_IMAGE_BUFF_SIZE];

void DVPDemo_Init(void);
void DVPDemo_Capture(void);

#ifdef __cplusplus
}
#endif

#endif

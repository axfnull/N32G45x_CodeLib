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
 * @file julia_fpu.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "julia_fpu.h"

/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

/**
 * @brief  Julia Calculation with FPU option disable
 * @param size_x screen width in pixel
 * @param size_y screen height in pixel
 * @param offset_x x offset for starting point
 * @param offset_y y offset for starting point
 * @param zoom Iteration zoom ratio
 * @param buffer pointer to output buffer where values are stored
 */
void GenerateJulia_fpu(uint16_t size_x,
                       uint16_t size_y,
                       uint16_t offset_x,
                       uint16_t offset_y,
                       uint16_t zoom,
                       uint8_t* buffer)
{
    float tmp1, tmp2;
    float num_real, num_img;
    float rayon;

    uint8_t i;
    uint16_t x, y;

    for (y = 0; y < size_y; y++)
    {
        for (x = 0; x < size_x; x++)
        {
            num_real = y - offset_y;
            num_real = num_real / zoom;
            num_img  = x - offset_x;
            num_img  = num_img / zoom;
            i        = 0;
            rayon    = 0;
            while ((i < ITERATION - 1) && (rayon < 4))
            {
                tmp1     = num_real * num_real;
                tmp2     = num_img * num_img;
                num_img  = 2 * num_real * num_img + IMG_CONSTANT;
                num_real = tmp1 - tmp2 + REAL_CONSTANT;
                rayon    = tmp1 + tmp2;
                i++;
            }
            /* Store the value in the buffer */
            buffer[x + y * size_x] = i;
        }
    }
}

/**
 * @}
 */
/**
 * @}
 */

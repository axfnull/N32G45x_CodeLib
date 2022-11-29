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
 * @file virtual_i2c.h
 * @author Nations
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __VIRTUAL_I2C_H__
#define __VIRTUAL_I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define VirI2C_Dir_W (uint8_t)0 // I2C read
#define VirI2C_Dir_R (uint8_t)1 // I2C write

#ifndef true
#define true (uint8_t)1
#endif

#ifndef false
#define false (uint8_t)0
#endif

#ifndef bool
#define bool uint8_t
#endif

#if 1
#define GC_SCL_PORT GPIOB
#define GC_SCL_PIN  GPIO_PIN_8
#define GC_SCL_H    GC_SCL_PORT->PBSC = GC_SCL_PIN
#define GC_SCL_L    GC_SCL_PORT->PBC = GC_SCL_PIN
#define GC_SCL_READ (GC_SCL_PORT->PID & GC_SCL_PIN)

#define GC_SDA_PORT GPIOB
#define GC_SDA_PIN  GPIO_PIN_9
#define GC_SDA_H    GC_SDA_PORT->PBSC = GC_SDA_PIN
#define GC_SDA_L    GC_SDA_PORT->PBC = GC_SDA_PIN
#define GC_SDA_READ (GC_SDA_PORT->PID & GC_SDA_PIN)

#else
#define GC_SCL_PORT GPIOA
#define GC_SCL_PIN  GPIO_PIN_4
#define GC_SCL_H    GC_SCL_PORT->PBSC = GC_SCL_PIN
#define GC_SCL_L    GC_SCL_PORT->PBC = GC_SCL_PIN
#define GC_SCL_READ (GC_SCL_PORT->PID & GC_SCL_PIN)

#define GC_SDA_PORT GPIOA
#define GC_SDA_PIN  GPIO_PIN_5
#define GC_SDA_H    GC_SDA_PORT->PBSC = GC_SDA_PIN
#define GC_SDA_L    GC_SDA_PORT->PBC = GC_SDA_PIN
#define GC_SDA_READ (GC_SDA_PORT->PID & GC_SDA_PIN)
#endif

void GC_VirI2CInit(void);

bool GC_WriteReg(uint8_t addr, uint8_t reg, uint8_t data);
bool GC_WriteRegBuf(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf);
bool GC_ReadReg(uint8_t addr, uint8_t reg, uint8_t* data);
bool GC_ReadRegBuf(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf);

#ifdef __cplusplus
}
#endif

#endif

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
 * @file virtual_i2c.c
 * @author Nations
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "virtual_i2c.h"

#define GC_VirI2C_Delay()  (DelayUs(2)) /*Normal I2C delay, about 2us*/

/**
 * @brief   Generate a start signal for I2C interface.
 * @param   None
 * @retval: Opretion result,ture or false
 */
static bool GC_VirI2C_Start(void)
{
    GC_SDA_H;
    GC_SCL_H;
    GC_VirI2C_Delay();
    
    if (!GC_SDA_READ)
        return false;
    
    GC_SDA_L;
    GC_VirI2C_Delay();
    if (GC_SDA_READ)
        return false;

    GC_SCL_L;
    GC_VirI2C_Delay();
    return true;
}

/**
 * @brief   Generate a stop signal for I2C interface.
 * @param   None
 * @retval: None
 */
static void GC_VirI2C_Stop(void)
{
    GC_SCL_L;
    GC_VirI2C_Delay();
    GC_SDA_L;
    GC_VirI2C_Delay();
    GC_SCL_H;
    GC_VirI2C_Delay();
    GC_SDA_H;
    GC_VirI2C_Delay();
}

/**
 * @brief   Send a ACK signal for I2C interface.
 * @param   None
 * @retval: None
 */
static void GC_VirI2C_Ack(void)
{
    GC_SCL_L;
    GC_VirI2C_Delay();
    GC_SDA_L;
    GC_VirI2C_Delay();
    GC_SCL_H;
    GC_VirI2C_Delay();
    GC_SCL_L;
    GC_VirI2C_Delay();
}

/**
 * @brief   Send a No ACK signal for I2C interface.
 * @param   None
 * @retval: None
 */
static void GC_VirI2C_NoAck(void)
{
    GC_SCL_L;
    GC_VirI2C_Delay();
    GC_SDA_H;
    GC_VirI2C_Delay();
    GC_SCL_H;
    GC_VirI2C_Delay();
    GC_SCL_L;
    GC_VirI2C_Delay();
}

/**
 * @brief   Receive a ACK signal and check.
 * @param   None
 * @retval: Opretion result,ture or false
 */
static bool GC_VirI2C_WaitAck(void)
{
    GC_SCL_L;
    GC_VirI2C_Delay();
    GC_SDA_H;
    GC_VirI2C_Delay();
    GC_SCL_H;
    GC_VirI2C_Delay();
    if (GC_SDA_READ)
    {
        GC_SCL_L;
        return false;
    }
    GC_SCL_L;
    return true;
}

/**
 * @brief   Send a byte by I2C.
 * @param   None
 * @retval: None
 */
static void GC_VirI2C_SendByte(uint8_t byte)
{
    uint8_t i = 8;

    while (i--)
    {
        GC_SCL_L;
        GC_VirI2C_Delay();
        
        if (byte & 0x80)
            GC_SDA_H;
        else
            GC_SDA_L;

        byte <<= 1;

        GC_VirI2C_Delay();
        
        GC_SCL_H;
        GC_VirI2C_Delay();
    }
    
    GC_SCL_L;
    GC_VirI2C_Delay();
}

/**
 * @brief   Receive a byte by I2C.
 * @param   None
 * @retval: Opretion result,ture or false
 */
static uint8_t GC_VirI2C_ReceiveByte(void)
{
    uint8_t i    = 8;
    uint8_t byte = 0;

    while (i--)
    {
        byte <<= 1;
        
        GC_SCL_L;
        GC_VirI2C_Delay();
        GC_SDA_H;
        GC_VirI2C_Delay();
        GC_SCL_H;
        GC_VirI2C_Delay();

        if (GC_SDA_READ)
            byte |= 0x01;
        else
            byte &= 0xFE;
    }
    
    GC_SCL_L;
    GC_VirI2C_Delay();
    return byte;
}

/**
 * @brief   Init the port for vitual I2C interface.
 * @param   None
 * @retval: None
 */
void GC_VirI2CInit(void)
{
    GPIO_InitType GPIO_InitStruct;

    GPIO_InitStruct.Pin        = GC_SCL_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GC_SCL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin        = GC_SDA_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GC_SDA_PORT, &GPIO_InitStruct);

    GC_VirI2C_Stop();
}

/**
 * @brief   Write data into single register of slave.
 * @param   None
 * @retval: Opretion result,ture or false
 */
bool GC_WriteReg(uint8_t addr, uint8_t reg, uint8_t data)
{
    if (!GC_VirI2C_Start())
    {
        GC_VirI2C_Stop();
        return false;
    }

    GC_VirI2C_SendByte(addr | VirI2C_Dir_W);
    if (!GC_VirI2C_WaitAck())
    {
        GC_VirI2C_Stop();
        return false;
    }

    GC_VirI2C_SendByte(reg);
    if (!GC_VirI2C_WaitAck())
    {
        GC_VirI2C_Stop();
        return false;
    }

    GC_VirI2C_SendByte(data);
    if (!GC_VirI2C_WaitAck())
    {
        GC_VirI2C_Stop();
        return false;
    }

    GC_VirI2C_Stop();

    return true;
}

/**
 * @brief   Write some datas into continuous registers of slave.
 * @param   None
 * @retval: Opretion result,ture or false
 */
bool GC_WriteRegBuf(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
    int i = 0;

    if (!GC_VirI2C_Start())
    {
        GC_VirI2C_Stop();
        return false;
    }

    GC_VirI2C_SendByte(addr | VirI2C_Dir_W);
    if (!GC_VirI2C_WaitAck())
    {
        GC_VirI2C_Stop();
        return false;
    }

    GC_VirI2C_SendByte(reg);
    GC_VirI2C_WaitAck();

    for (i = 0; i < len; i++)
    {
        GC_VirI2C_SendByte(buf[i]);
        if (!GC_VirI2C_WaitAck())
        {
            GC_VirI2C_Stop();
            return false;
        }
    }
    GC_VirI2C_Stop();

    return true;
}

/**
 * @brief   Read data from single registers of slave.
 * @param   None
 * @retval: Opretion result,ture or false
 */

bool GC_ReadReg(uint8_t addr, uint8_t reg, uint8_t* data)
{
    if (!GC_VirI2C_Start())
    {
        GC_VirI2C_Stop();
        return false;
    }

    GC_VirI2C_SendByte(addr | VirI2C_Dir_W);
    if (!GC_VirI2C_WaitAck())
    {
        GC_VirI2C_Stop();
        return false;
    }

    GC_VirI2C_SendByte(reg);
    if (!GC_VirI2C_WaitAck())
    {
        GC_VirI2C_Stop();
        return false;
    }

    GC_VirI2C_Start();
    GC_VirI2C_SendByte(addr | VirI2C_Dir_R);
    if (!GC_VirI2C_WaitAck())
    {
        GC_VirI2C_Stop();
        return false;
    }

    *data = GC_VirI2C_ReceiveByte();

    GC_VirI2C_NoAck();

    GC_VirI2C_Stop();

    return true;
}

/**
 * @brief   Read some datas from continuous registers of slave.
 * @param   None
 * @retval: Opretion result,ture or false
 */
bool GC_ReadRegBuf(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
    if (!GC_VirI2C_Start())
    {
        GC_VirI2C_Stop();
        return false;
    }

    GC_VirI2C_SendByte(addr | VirI2C_Dir_W);
    if (!GC_VirI2C_WaitAck())
    {
        GC_VirI2C_Stop();
        return false;
    }

    GC_VirI2C_SendByte(reg);
    if (!GC_VirI2C_WaitAck())
    {
        GC_VirI2C_Stop();
        return false;
    }

    GC_VirI2C_Start();
    GC_VirI2C_SendByte(addr | VirI2C_Dir_R);
    if (!GC_VirI2C_WaitAck())
    {
        GC_VirI2C_Stop();
        return false;
    }
    
    while (len)
    {
        *buf = GC_VirI2C_ReceiveByte();
        if (len == 1)
            GC_VirI2C_NoAck();
        else
            GC_VirI2C_Ack();
        buf++;
        len--;
    }
    GC_VirI2C_Stop();

    return true;
}

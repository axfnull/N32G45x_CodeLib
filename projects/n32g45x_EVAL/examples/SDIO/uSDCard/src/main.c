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
 * @file main.c
 * @author Nations
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "n32g45x.h"
#include "n32g45x_i2c.h"
#include "n32g45x_sdio.h"
#include "sdio_sdcard.h"
#include "log.h"

/** @addtogroup N32G45X_StdPeriph_Examples
 * @{
 */

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;
#define BLOCK_SIZE 512

#pragma pack(4)
u32 Buffer_Block_Tx[BLOCK_SIZE / 4], Buffer_Block_Rx[BLOCK_SIZE / 4];
u32 Buffer_MulBlock_Tx[BLOCK_SIZE], Buffer_MulBlock_Rx[BLOCK_SIZE];
#pragma pack()

#define Buffer_Block_Tx ((uint8_t*)Buffer_Block_Tx)
#define Buffer_Block_Rx ((uint8_t*)Buffer_Block_Rx)

#define Buffer_MulBlock_Tx ((uint8_t*)Buffer_MulBlock_Tx)
#define Buffer_MulBlock_Rx ((uint8_t*)Buffer_MulBlock_Rx)

#define MUL_BLOCK_RW

#ifdef MUL_BLOCK_RW
#define Buf_TX  Buffer_MulBlock_Tx
#define Buf_RX  Buffer_MulBlock_Rx
#define Buf_Len (BLOCK_SIZE * 4)
#else
#define Buf_TX  Buffer_Block_Tx
#define Buf_RX  Buffer_Block_Rx
#define Buf_Len BLOCK_SIZE
#endif

volatile TestStatus TransferStatus1 = FAILED;
SD_Error Status                     = SD_OK;

TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
void Memset(void* s, uint8_t c, uint32_t count);
void SD_Info(SD_CardInfo * info);
void dataShow(uint8_t* pBuf, uint16_t DataSize);

/**
 * @brief  Fills buffer with user predefined data.
 * @param pBuffer pointer on the Buffer to fill
 * @param BufferLength size of the buffer to fill
 * @param Offset first value to fill on the Buffer
 */
void Fill_Buffer(uint8_t* pBuffer, uint32_t BufferLength, uint32_t Offset)
{
    uint16_t index = 0;

    /* Put in global buffer same values */
    for (index = 0; index < BufferLength; index++)
    {
        pBuffer[index] = index + Offset;
    }
}

/**
 * @brief   Main program
 */
int main(void)
{
    uint32_t testResult = 0;
    
    log_init();
    Memset(Buf_RX, 0x00, Buf_Len);
    Fill_Buffer(Buf_TX, Buf_Len, 0x30);
    
    log_debug("------ This is a SDI/O Demo ------\r\n");

    Status = SD_Init(0, 3, 4);
    if (Status != SD_OK)
    {
        log_debug("SD Card initialization failed!\r\n");
        return testResult;
    }
    SD_Info(&SDCardInfo);
    dataShow(Buf_TX, Buf_Len);
    
    // sdio write
#ifdef MUL_BLOCK_RW
    Status = SD_WriteMultiBlocks(Buf_TX, 0x00, BLOCK_SIZE, 4);
#else
    Status = SD_WriteBlock(Buf_TX, 0x00, BLOCK_SIZE);
#endif
    
    Status = SD_WaitWriteOperation();
    while (SD_GetStatus() != SD_TRANSFER_OK)
        ;
    if (Status != SD_OK)
    {
        log_debug("SD Card write block failed!\r\n");
        return testResult;
    }

    // sdio read
#ifdef MUL_BLOCK_RW
    Status = SD_ReadMultiBlocks(Buf_RX, 0x00, BLOCK_SIZE, 4);
#else
    Status = SD_ReadBlock(Buf_RX, 0x00, BLOCK_SIZE);
#endif
    
    Status = SD_WaitReadOperation();
    while (SD_GetStatus() != SD_TRANSFER_OK)
        ;
    if (Status != SD_OK)
    {
        log_debug("SD Card read block failed!\r\n");
        return testResult;
    }
    
    dataShow(Buf_RX, Buf_Len);
    
    // compare read date with send data,the same test pass,otherwise test fail
    TransferStatus1 = Buffercmp(Buf_TX, Buf_RX, Buf_Len);
    if (TransferStatus1 != PASSED)
    {
        log_debug("SD Card r/w data check failed!\r\n");
        return testResult;
    }
    else
        log_debug("SD Card r/w data check success!\r\n");
    
    while (1)
    {
    }
}

/**
 * @brief  Compares two buffers.
 * @param  pBuffer, pBuffer1: buffers to be compared.
 * @param BufferLength buffer's length
 * @return PASSED: pBuffer identical to pBuffer1
 *         FAILED: pBuffer differs from pBuffer1
 */
TestStatus Buffercmp(uint8_t* pBuffer, uint8_t* pBuffer1, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer != *pBuffer1)
        {
            return FAILED;
        }

        pBuffer++;
        pBuffer1++;
    }

    return PASSED;
}

/**
 * @brief memery set a value
 * @param s source
 * @param c value
 * @param count number
 * @return pointer of the memery
 */
void Memset(void* s, uint8_t c, uint32_t count)
{
    char* xs = (char*)s;

    while (count--) // clear 17byte buffer
    {
        *xs++ = c;
    }

    return;
}

void SD_Info(SD_CardInfo* info)
{
    log_debug("SD Card initialization success!\r\n");
    log_debug("*CardType            is: %d\r\n", info->CardType);
    log_debug("*CardCapacity        is: %lld\r\n", info->CardCapacity);
    log_debug("*CardBlockSize       is: %d\r\n", info->CardBlockSize);
    log_debug("*RCA                 is: %d\r\n", info->RCA);
    log_debug("*Manufacture(MID)    is: %d\r\n", info->SD_cid.ManufacturerID);
    log_debug("*OEM/Appli(OID)      is: %d\r\n", info->SD_cid.OEM_AppliID);
    log_debug("*Product Name(PNM)   is: %d\r\n", info->SD_cid.ProdName1);
    log_debug("*Serial Number(PSN)  is: %x\r\n", info->SD_cid.ProdSN);
    log_debug("*Manu Date COde(MDT) is: %x\r\n", info->SD_cid.ManufactDate);
    log_debug("*Card SysSpecVersion is: %d\r\n", info->SD_csd.SysSpecVersion);
    log_debug("*Card MaxBusClkFrec  is: %d\r\n", info->SD_csd.MaxBusClkFrec);
    log_debug("*Card MaxRdBlockLen  is: %d\r\n", info->SD_csd.RdBlockLen);
    log_debug("*Card RdCurrent VDD  is: %d -> %d\r\n", info->SD_csd.MaxRdCurrentVDDMin, info->SD_csd.MaxRdCurrentVDDMax);
    log_debug("*Card WrSpeedFact    is: %d\r\n", info->SD_csd.WrSpeedFact);
    log_debug("*Card MaxWrBlockLen  is: %d\r\n", info->SD_csd.MaxWrBlockLen);
    log_debug("*Card WrCurrent VDD  is: %d -> %d\r\n", info->SD_csd.MaxWrCurrentVDDMin, info->SD_csd.MaxWrCurrentVDDMax);
}
    
void dataShow(uint8_t* pBuf, uint16_t DataSize)
{
    for(uint16_t i = 0; i < DataSize; i++)
    {
        if(i % 8 == 0)
        {
            log_debug("\r\n:");
        }
        log_debug("  0x%0.2X", *(pBuf + i));
    }
    log_debug("\r\n\r\n");
}
    
/**
 * @}
 */

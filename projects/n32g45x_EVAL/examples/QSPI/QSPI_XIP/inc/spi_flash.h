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
 * @file spi_flash.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef     _SPI_FLASH__
#define     _SPI_FLASH__

#define TEST_ADDR                       0x000000

/** ×´Ì¬¼Ä´æÆ÷ */
#define W25X_WriteEnable        0x06 
#define W25X_WriteDisable       0x04 
#define W25X_ReadStatusReg1     0x05 
#define W25X_ReadStatusReg2     0x35 
#define W25X_ReadStatusReg3     0x15 
#define W25X_WriteStatusReg1    0x01 
#define W25X_WriteStatusReg2    0x31 
#define W25X_WriteStatusReg3    0x11 

/** ¶ÁÃüÁî */
#define W25X_ReadData           0x03    //read
#define W25X_FastReadData       0x0B    //fast read
#define W25X_FastReadDual       0x3B    //Dual read
#define W25X_ReadQuadOutput     0x6B    //Quad read

/** ²Á³ýÃüÁî */
#define W25X_PageErase          0x81    //256 Byte
#define W25X_SectorErase        0x20    //4K Byte
//#define W25X_BlockErase       0x52    //32K Byte
#define W25X_BlockErase         0xD8    //64K Byte
#define W25X_ChipErase          0xC7    //0x60 Whole 

/** Ð´ÃüÁî */
#define W25X_PageProgram        0x02    //page program
#define W25X_DualPageProgram    0xA2    //Dual page program
#define W25X_QuadPageProgram    0x32    //Duad page program

/** ÆäËû */
#define W25X_PowerDown          0xB9 
#define W25X_ReleasePowerDown   0xAB 
#define W25X_DeviceID           0xAB    
#define W25X_ManufactDeviceID   0x90    //Read Manufacture ID 0x92/ 0x94
#define W25X_JedecDeviceID      0x9F    //Read device ID
#define W25X_UniqueID           0x4B    //Read Unique ID
#define W25X_Enable4ByteAddr    0xB7
#define W25X_Exit4ByteAddr      0xE9
#define W25X_SetReadParam       0xC0 
#define W25X_EnterQPIMode       0x38
#define W25X_ExitQPIMode        0xFF

#define W25X_VSR_WriteEnable    0x50







extern void W25Qxx_Init(void);
extern void QSPI_Test(void);
















#endif

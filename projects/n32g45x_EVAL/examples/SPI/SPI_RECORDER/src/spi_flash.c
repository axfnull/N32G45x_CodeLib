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
 * @file spi_flash.c
 * @author Nations 
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "spi_flash.h"

/** @addtogroup Utilities
 * @{
 */

/** @addtogroup
 * @{
 */

/** @addtogroup Common
 * @{
 */

/** @addtogroup SPI_FLASH
 * @brief
 * @{
 */

/** @addtogroup SPI_FLASH_Private_Types
 * @{
 */
/**
 * @}
 */

/** @addtogroup SPI_FLASH_Private_Defines
 * @{
 */
/**
 * @}
 */

/** @addtogroup SPI_FLASH_Private_Macros
 * @{
 */
/**
 * @}
 */

/** @addtogroup SPI_FLASH_Private_Variables
 * @{
 */
/**
 * @}
 */

/** @addtogroup SPI_FLASH_Private_Function_Prototypes
 * @{
 */
/**
 * @}
 */

/** @addtogroup SPI_FLASH_Private_Functions
 * @{
 */
 
/**
 * @brief  DeInitializes the peripherals used by the SPI FLASH driver.
 */
void sFLASH_LowLevel_DeInit(void)
{
    GPIO_InitType GPIO_InitStructure;

    /*!< Disable the sFLASH_SPI  */
    SPI_Enable(sFLASH_SPI, DISABLE);

    /*!< DeInitializes the sFLASH_SPI */
    SPI_I2S_DeInit(sFLASH_SPI);

    /*!< sFLASH_SPI Periph clock disable */
    RCC_EnableAPB2PeriphClk(sFLASH_SPI_CLK, DISABLE);

    /*!< Configure sFLASH_SPI pins: SCK */
    GPIO_InitStructure.Pin       = sFLASH_SPI_SCK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitPeripheral(sFLASH_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

    /*!< Configure sFLASH_SPI pins: MISO */
    GPIO_InitStructure.Pin = sFLASH_SPI_MISO_PIN;
    GPIO_InitPeripheral(sFLASH_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

    /*!< Configure sFLASH_SPI pins: MOSI */
    GPIO_InitStructure.Pin = sFLASH_SPI_MOSI_PIN;
    GPIO_InitPeripheral(sFLASH_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

    /*!< Configure sFLASH_CS_PIN pin: sFLASH Card CS pin */
    GPIO_InitStructure.Pin = sFLASH_CS_PIN;     //SPI1_CS 被用作 VS1053B 的 DREQ，所以用 PC4 用作 FLASH_CS
    GPIO_InitPeripheral(sFLASH_CS_GPIO_PORT, &GPIO_InitStructure);
}

/**
 * @brief  Initializes the peripherals used by the SPI FLASH driver.
 */
void sFLASH_LowLevel_Init(void)
{
    GPIO_InitType GPIO_InitStructure;

    /*!< sFLASH_SPI_CS_GPIO, sFLASH_SPI_MOSI_GPIO, sFLASH_SPI_MISO_GPIO
         and sFLASH_SPI_SCK_GPIO Periph clock enable */
    RCC_EnableAPB2PeriphClk(
        sFLASH_CS_GPIO_CLK | sFLASH_SPI_MOSI_GPIO_CLK | sFLASH_SPI_MISO_GPIO_CLK | sFLASH_SPI_SCK_GPIO_CLK, ENABLE);

    /*!< sFLASH_SPI Periph clock enable */
    RCC_EnableAPB2PeriphClk(sFLASH_SPI_CLK, ENABLE);

    /*!< Configure sFLASH_SPI pins: SCK */
    GPIO_InitStructure.Pin        = sFLASH_SPI_SCK_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitPeripheral(sFLASH_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

    /*!< Configure sFLASH_SPI pins: MOSI */
    GPIO_InitStructure.Pin = sFLASH_SPI_MOSI_PIN;
    GPIO_InitPeripheral(sFLASH_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

    /*!< Configure sFLASH_SPI pins: MISO */
    GPIO_InitStructure.Pin       = sFLASH_SPI_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitPeripheral(sFLASH_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

    /*!< Configure sFLASH_CS_PIN pin: sFLASH Card CS pin */
    GPIO_InitStructure.Pin       = sFLASH_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitPeripheral(sFLASH_CS_GPIO_PORT, &GPIO_InitStructure);
}

/**
 * @brief  DeInitializes the peripherals used by the SPI FLASH driver.
 */
void sFLASH_DeInit(void)
{
    sFLASH_LowLevel_DeInit();
}

/**
 * @brief  Initializes the peripherals used by the SPI FLASH driver.
 */
void sFLASH_Init(void)
{
    SPI_InitType SPI_InitStructure;

    sFLASH_LowLevel_Init();

    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();

    /*!< SPI configuration */
    SPI_InitStructure.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
    SPI_InitStructure.SpiMode       = SPI_MODE_MASTER;
    SPI_InitStructure.DataLen       = SPI_DATA_SIZE_8BITS;
    SPI_InitStructure.CLKPOL        = SPI_CLKPOL_HIGH;
    SPI_InitStructure.CLKPHA        = SPI_CLKPHA_SECOND_EDGE;
    SPI_InitStructure.NSS           = SPI_NSS_SOFT;

    SPI_InitStructure.BaudRatePres = SPI_BR_PRESCALER_4;

    SPI_InitStructure.FirstBit = SPI_FB_MSB;
    SPI_InitStructure.CRCPoly  = 7;
    SPI_Init(sFLASH_SPI, &SPI_InitStructure);

    /*!< Enable the sFLASH_SPI  */
    SPI_Enable(sFLASH_SPI, ENABLE);
}

/**
 * @brief  set spi speed
 * @param  SPI_BaudRatePrescaler
 * @return None
 */

void sFLASH_SetSpeed(uint8_t SpeedSet)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
    sFLASH_SPI->CTRL1&=0XFFC7; 
    sFLASH_SPI->CTRL1|=SpeedSet;
    SPI_Enable(sFLASH_SPI,ENABLE); 
}

/**
 * @brief  Erases the specified FLASH sector.
 * @param SectorAddr address of the sector to erase.
 */
void sFLASH_EraseSector(uint32_t SectorAddr)
{
    /*!< Send write enable instruction */
    sFLASH_WriteEnable();

    /*!< Sector Erase */
    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();
    /*!< Send Sector Erase instruction */
    sFLASH_SendByte(sFLASH_CMD_SE);
    /*!< Send SectorAddr high nibble address byte */
    sFLASH_SendByte((SectorAddr & 0xFF0000) >> 16);
    /*!< Send SectorAddr medium nibble address byte */
    sFLASH_SendByte((SectorAddr & 0xFF00) >> 8);
    /*!< Send SectorAddr low nibble address byte */
    sFLASH_SendByte(SectorAddr & 0xFF);
    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();

    /*!< Wait the end of Flash writing */
    sFLASH_WaitForWriteEnd();
}

/**
 * @brief  Erases the entire FLASH.
 */
void sFLASH_EraseBulk(void)
{
    /*!< Send write enable instruction */
    sFLASH_WriteEnable();

    /*!< Bulk Erase */
    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();
    /*!< Send Bulk Erase instruction  */
    sFLASH_SendByte(sFLASH_CMD_BE);
    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();

    /*!< Wait the end of Flash writing */
    sFLASH_WaitForWriteEnd();
}

/**
 * @brief  Writes more than one byte to the FLASH with a single WRITE cycle
 *         (Page WRITE sequence).
 * @note   The number of byte can't exceed the FLASH page size.
 * @param pBuffer pointer to the buffer  containing the data to be written
 *         to the FLASH.
 * @param WriteAddr FLASH's internal address to write to.
 * @param NumByteToWrite number of bytes to write to the FLASH, must be equal
 *         or less than "sFLASH_PAGESIZE" value.
 */
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    /*!< Enable the write access to the FLASH */
    sFLASH_WriteEnable();

    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();
    /*!< Send "Write to Memory " instruction */
    sFLASH_SendByte(sFLASH_CMD_WRITE);
    /*!< Send WriteAddr high nibble address byte to write to */
    sFLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
    /*!< Send WriteAddr medium nibble address byte to write to */
    sFLASH_SendByte((WriteAddr & 0xFF00) >> 8);
    /*!< Send WriteAddr low nibble address byte to write to */
    sFLASH_SendByte(WriteAddr & 0xFF);

    /*!< while there is data to be written on the FLASH */
    while (NumByteToWrite--)
    {
        /*!< Send the current byte */
        sFLASH_SendByte(*pBuffer);
        /*!< Point on the next byte to be written */
        pBuffer++;
    }

    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();

    /*!< Wait the end of Flash writing */
    sFLASH_WaitForWriteEnd();
}

/**
 * @brief  Writes block of data to the FLASH. In this function, the number of
 *         WRITE cycles are reduced, using Page WRITE sequence.
 * @param pBuffer pointer to the buffer  containing the data to be written
 *         to the FLASH.
 * @param WriteAddr FLASH's internal address to write to.
 * @param NumByteToWrite number of bytes to write to the FLASH.
 */
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

    Addr        = WriteAddr % sFLASH_SPI_PAGESIZE;
    count       = sFLASH_SPI_PAGESIZE - Addr;
    NumOfPage   = NumByteToWrite / sFLASH_SPI_PAGESIZE;
    NumOfSingle = NumByteToWrite % sFLASH_SPI_PAGESIZE;
        
    if (Addr == 0) /*!< WriteAddr is sFLASH_PAGESIZE aligned  */
    {
        if (NumOfPage == 0) /*!< NumByteToWrite < sFLASH_PAGESIZE */
        {
            sFLASH_WritePage(pBuffer, WriteAddr, NumByteToWrite);
        }
        else /*!< NumByteToWrite > sFLASH_PAGESIZE */
        {
            while (NumOfPage--)
            {
                sFLASH_WritePage(pBuffer, WriteAddr, sFLASH_SPI_PAGESIZE);
                WriteAddr += sFLASH_SPI_PAGESIZE;
                pBuffer += sFLASH_SPI_PAGESIZE;
            }

            sFLASH_WritePage(pBuffer, WriteAddr, NumOfSingle);
        }
    }
    else /*!< WriteAddr is not sFLASH_PAGESIZE aligned  */
    {
        if (NumOfPage == 0) /*!< NumByteToWrite < sFLASH_PAGESIZE */
        {
            if (NumOfSingle > count) /*!< (NumByteToWrite + WriteAddr) > sFLASH_PAGESIZE */
            {
                temp = NumOfSingle - count;

                sFLASH_WritePage(pBuffer, WriteAddr, count);
                WriteAddr += count;
                pBuffer += count;

                sFLASH_WritePage(pBuffer, WriteAddr, temp);
            }
            else
            {
                sFLASH_WritePage(pBuffer, WriteAddr, NumByteToWrite);
            }
        }
        else /*!< NumByteToWrite > sFLASH_PAGESIZE */
        {
            NumByteToWrite -= count;
            NumOfPage   = NumByteToWrite / sFLASH_SPI_PAGESIZE;
            NumOfSingle = NumByteToWrite % sFLASH_SPI_PAGESIZE;

            sFLASH_WritePage(pBuffer, WriteAddr, count);
            WriteAddr += count;
            pBuffer += count;

            while (NumOfPage--)
            {
                sFLASH_WritePage(pBuffer, WriteAddr, sFLASH_SPI_PAGESIZE);
                WriteAddr += sFLASH_SPI_PAGESIZE;
                pBuffer += sFLASH_SPI_PAGESIZE;
            }
            if (NumOfSingle != 0)
            {
                sFLASH_WritePage(pBuffer, WriteAddr, NumOfSingle);
            }
        }
    }
}

/**
 * @brief  Reads a block of data from the FLASH.
 * @param pBuffer pointer to the buffer that receives the data read from the FLASH.
 * @param ReadAddr FLASH's internal address to read from.
 * @param NumByteToRead number of bytes to read from the FLASH.
 */
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();

    /*!< Send "Read from Memory " instruction */
    sFLASH_SendByte(sFLASH_CMD_READ);

    /*!< Send ReadAddr high nibble address byte to read from */
    sFLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
    /*!< Send ReadAddr medium nibble address byte to read from */
    sFLASH_SendByte((ReadAddr & 0xFF00) >> 8);
    /*!< Send ReadAddr low nibble address byte to read from */
    sFLASH_SendByte(ReadAddr & 0xFF);

    while (NumByteToRead--) /*!< while there is data to be read */
    {
        /*!< Read a byte from the FLASH */
        *pBuffer = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
        /*!< Point to the next location where the byte read will be saved */
        pBuffer++;
    }

    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();
}

/**
 * @brief  Reads FLASH identification.
 * @return FLASH identification
 */
uint32_t sFLASH_ReadID(void)
{
    uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();

    /*!< Send "RDID " instruction */
    sFLASH_SendByte(0x9F);
    /*!< Read a byte from the FLASH */
    Temp0 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

    /*!< Read a byte from the FLASH */
    Temp1 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

    /*!< Read a byte from the FLASH */
    Temp2 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();

    Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;

    return Temp;
}

/**
 * @brief  Initiates a read data byte (READ) sequence from the Flash.
 *   This is done by driving the /CS line low to select the device, then the READ
 *   instruction is transmitted followed by 3 bytes address. This function exit
 *   and keep the /CS line low, so the Flash still being selected. With this
 *   technique the whole content of the Flash is read with a single READ instruction.
 * @param ReadAddr FLASH's internal address to read from.
 */
void sFLASH_StartReadSequence(uint32_t ReadAddr)
{
    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();

    /*!< Send "Read from Memory " instruction */
    sFLASH_SendByte(sFLASH_CMD_READ);

    /*!< Send the 24-bit address of the address to read from -------------------*/
    /*!< Send ReadAddr high nibble address byte */
    sFLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
    /*!< Send ReadAddr medium nibble address byte */
    sFLASH_SendByte((ReadAddr & 0xFF00) >> 8);
    /*!< Send ReadAddr low nibble address byte */
    sFLASH_SendByte(ReadAddr & 0xFF);
}

/**
 * @brief  Reads a byte from the SPI Flash.
 * @note   This function must be used only if the Start_Read_Sequence function
 *         has been previously called.
 * @return Byte Read from the SPI Flash.
 */
uint8_t sFLASH_ReadByte(void)
{
    return (sFLASH_SendByte(sFLASH_DUMMY_BYTE));
}

/**
 * @brief  Sends a byte through the SPI interface and return the byte received
 *         from the SPI bus.
 * @param byte byte to send.
 * @return The value of the received byte.
 */
uint8_t sFLASH_SendByte(uint8_t byte)
{
    /*!< Loop while DAT register in not emplty */
    while (SPI_I2S_GetStatus(sFLASH_SPI, SPI_I2S_TE_FLAG) == RESET)
        ;

    /*!< Send byte through the SPI1 peripheral */
    SPI_I2S_TransmitData(sFLASH_SPI, byte);

    /*!< Wait to receive a byte */
    while (SPI_I2S_GetStatus(sFLASH_SPI, SPI_I2S_RNE_FLAG) == RESET)
        ;

    /*!< Return the byte read from the SPI bus */
    return SPI_I2S_ReceiveData(sFLASH_SPI);
}

/**
 * @brief  Sends a Half Word through the SPI interface and return the Half Word
 *         received from the SPI bus.
 * @param HalfWord Half Word to send.
 * @return The value of the received Half Word.
 */
uint16_t sFLASH_SendHalfWord(uint16_t HalfWord)
{
    /*!< Loop while DAT register in not emplty */
    while (SPI_I2S_GetStatus(sFLASH_SPI, SPI_I2S_TE_FLAG) == RESET)
        ;

    /*!< Send Half Word through the sFLASH peripheral */
    SPI_I2S_TransmitData(sFLASH_SPI, HalfWord);

    /*!< Wait to receive a Half Word */
    while (SPI_I2S_GetStatus(sFLASH_SPI, SPI_I2S_RNE_FLAG) == RESET)
        ;

    /*!< Return the Half Word read from the SPI bus */
    return SPI_I2S_ReceiveData(sFLASH_SPI);
}

/**
 * @brief  Enables the write access to the FLASH.
 */
void sFLASH_WriteEnable(void)
{
    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();

    /*!< Send "Write Enable" instruction */
    sFLASH_SendByte(sFLASH_CMD_WREN);

    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();
}

/**
 * @brief  Polls the status of the Write In Progress (WIP) flag in the FLASH's
 *         status register and loop until write opertaion has completed.
 */
void sFLASH_WaitForWriteEnd(void)
{
    uint8_t flashstatus = 0;

    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();

    /*!< Send "Read Status Register" instruction */
    sFLASH_SendByte(sFLASH_CMD_RDSR);

    /*!< Loop as long as the memory is busy with a write cycle */
    do
    {
        /*!< Send a dummy byte to generate the clock needed by the FLASH
        and put the value of the status register in FLASH_STS variable */
        flashstatus = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

    } while ((flashstatus & sFLASH_WIP_FLAG) == SET); /* Write in progress */

    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();
}

/**
 * @brief  read back satus resigter data
 */
uint8_t W25QXX_ReadSR(void)   
{  
    uint8_t byte=0;  
    /*!< Select the FLASH: Chip Select low */ 
    sFLASH_CS_LOW(); 
    /*!< Send "Read Status Register" instruction */
    sFLASH_SendByte(sFLASH_CMD_RDSR); 
    /*!< Read a byte from the FLASH */ 
    byte=sFLASH_SendByte(0Xff);
    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();  
    return byte;   
} 

/**
 * @brief  wait w25qxx free
 */
void W25QXX_Wait_Busy(void)   
{
    while((W25QXX_ReadSR()&0x01)==0x01);
}  

/**
 * @brief  erase a sector, erase one sector at least 150ms
 * @param Dst_Addr: addr of erase sector
 */
void W25QXX_Erase_Sector(uint32_t Dst_Addr)
{
    /*!< Send write enable instruction */
    sFLASH_WriteEnable();    
    /*!< Select the FLASH: Chip Select low */   
    sFLASH_CS_LOW();
    /*!< Send "erase sector command " instruction */
    sFLASH_SendByte(sFLASH_CMD_ERASE_SECTOR);
    /*!< Send 24bit address */
    sFLASH_SendByte((uint8_t)((Dst_Addr) >> 16));   
    sFLASH_SendByte((uint8_t)((Dst_Addr) >> 8));
    sFLASH_SendByte((uint8_t)Dst_Addr);
    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();
    /*!< wait w25qxx free >*/
    W25QXX_Wait_Busy();
}

/**
 * @brief  erase a block
 * @param Dst_Addr: addr of erase block
 */
void W25QXX_Erase_Block(uint32_t Dst_Addr)
{
    /*!< Send write enable instruction */
    sFLASH_WriteEnable();
    /*!< Select the FLASH: Chip Select low */ 
    sFLASH_CS_LOW();                                
    /*!< Send "erase block command " instruction */
    sFLASH_SendByte(sFLASH_CMD_ERASE_BLOCK);
    /*!< Send 24bit address */
    sFLASH_SendByte((uint8_t)((Dst_Addr) >> 16));
    sFLASH_SendByte((uint8_t)((Dst_Addr) >> 8));
    sFLASH_SendByte((uint8_t)Dst_Addr);
    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();
    /*!< wait w25qxx free >*/
    W25QXX_Wait_Busy();
}

/**
 * @brief  erase chip
 * @param Dst_Addr: addr of erase block
 */
void FLASH_ChipErase(void)
{
    /*!< wait w25qxx free >*/
    W25QXX_Wait_Busy();
    /*!< Send write enable instruction */
    sFLASH_WriteEnable();
    /*!< Select the FLASH: Chip Select low */ 
    sFLASH_CS_LOW();
    /*!< Send "erase chip command " instruction */
    sFLASH_SendByte(sFLASH_CMD_BE); //发送命令
    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();
}

/**
 * @brief  write  more than one byte to the FLASH with no check 
 * @param  pBuffer pointer to the buffer  containing the data to be written
 *         to the FLASH.
 * @param  WriteAddr FLASH's internal address to write to.
 * @param  NumByteToWrite number of bytes to write to the FLASH.
 */
void W25QXX_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint16_t pageremain;

    pageremain = 256 - WriteAddr % 256;

    if(NumByteToWrite <= pageremain)
    {
        pageremain = NumByteToWrite;
    }

    while(1)
    {
        sFLASH_WritePage(pBuffer, WriteAddr, pageremain);

        if(NumByteToWrite == pageremain)
        {
            break;//
        }
        else //NumByteToWrite>pageremain
        {
            pBuffer += pageremain;
            WriteAddr += pageremain;
            NumByteToWrite -= pageremain;

            if(NumByteToWrite > 256)
            {
                pageremain = 256;
            }
            else
            {
                pageremain = NumByteToWrite;
            }
        }
    }
    W25QXX_Wait_Busy();
}

/**
 * @brief  Writes more than one byte to the FLASH with check
 * @param pBuffer pointer to the buffer  containing the data to be written
 *         to the FLASH.
 * @param WriteAddr FLASH's internal address to write to.
 * @param NumByteToWrite number of bytes to write to the FLASH.
 */
uint8_t W25QXX_BUFFER[4096];
void W25QXX_Write(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint32_t secpos;
    uint16_t secoff;
    uint16_t secremain;
    uint16_t i;
    uint8_t * W25QXX_BUF;

    W25QXX_BUF = W25QXX_BUFFER;
    secpos = WriteAddr / 4096;
    secoff = WriteAddr % 4096;
    secremain = 4096 - secoff;

    if(NumByteToWrite <= secremain)
    {
        secremain = NumByteToWrite;
    }

    while(1)
    {
        sFLASH_ReadBuffer(W25QXX_BUF, WriteAddr, secremain);

        for(i = 0; i < secremain; i++)
        {
            if(W25QXX_BUF[i] != 0XFF)
                break;//need erase first
        }

        if(i < secremain)
        {
            sFLASH_ReadBuffer(W25QXX_BUF, secpos * 4096, 4096); //read back a sector data
            W25QXX_Erase_Sector(secpos);             //erase the sector

            for(i = 0; i < secremain; i++)
            {
                W25QXX_BUF[i + secoff] = pBuffer[i];
            }

            W25QXX_Write_NoCheck(W25QXX_BUF, secpos * 4096, 4096);
        }
        else
        {
            W25QXX_Write_NoCheck(pBuffer, WriteAddr, secremain);
        }

        if(NumByteToWrite == secremain)
        {
            break;
        }
        else
        {
            secpos++;
            secoff = 0;

            pBuffer += secremain; 
            WriteAddr += secremain;
            NumByteToWrite -= secremain;

            if(NumByteToWrite > 4096)
            {
                secremain = 4096;
            }
            else
            {
                secremain = NumByteToWrite;
            }
        }
    }

    W25QXX_Wait_Busy();
}

/**
 * @}
 */

/**
 * @}
 */

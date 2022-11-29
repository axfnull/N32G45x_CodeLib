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
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include    "SPI_Flash.h"
#include    "qspi_cfg.h"
#include    <stdio.h>
#include    <string.h>

/** N32G45x include */
#include    "n32g45x.h"
#include    "log.h"

/** User application include */





/** ---------------------------------------------------------------------------------------------------- */
/** ---------------------------------------------------------------------------------------------------- */

uint32_t    QSPI_TxBuf[256] = {0}, QSPI_RxBuf[256] = {0};


/**================================================================================
        delay function
 ================================================================================*/
static void Delay(uint32_t count)
{
    for (; count > 0; count--)
        ;
}

static void m_delay_ms(uint32_t ms)
{
    for(; ms!=0; ms--)
    {
        Delay(14400);
    }
}
/**================================================================================
Name:   wait for the busy of QSPI
 ================================================================================*/
static uint32_t WaitQSPI_Busy(void)
{
    uint32_t timeout = 0;
    
    Delay(5);
    while (GetQspiBusyStatus())
    {
        if(++timeout >= 20)
        {
            break;
        }
        Delay(50);
    }
    return 0xff;
}


/**================================================================================
        read status register1 of flash, wait for operating completely
 ================================================================================*/
static void QspiFlashReadStatus(void)
{
    uint32_t    tx_buf[2] = {0x05, 0x00};
    uint32_t    rx_buf[2];
    uint32_t    timeout = 0;

    do
    {
        m_delay_ms(1);
    
        QspiSendAndGetWords(tx_buf, rx_buf, 2);     
//        QspiSendWordAndGetWords(0x05, sr, 0);
//        QspiSendWordAndGetWords(0xff, sr, 1);
        if(++timeout >= 200)
        {
            break;
        }
    } while ((rx_buf[1] & 0x01) != 0x00);
}


/**================================================================================
        enable Write flash
 ================================================================================*/
static void QspiFlashWriteEnable(uint32_t Cmd)
{
    u32 temp;

    QspiSendWordAndGetWords(Cmd, &temp, 1);
    WaitQSPI_Busy();
}


/**================================================================================
        enable QE
 ================================================================================*/
static void QspiFlashEnableQE(void)
{
    uint32_t    temp[3] = {0x35, 0xff, 0xff};
    
    // read status register 
    QspiSendAndGetWords(temp, temp, 2);
    WaitQSPI_Busy();
    
    // enable QE if disable
    if(!(temp[1] & 0x02))
    {
        temp[0] = 0x01;
        temp[1] = 0x02;
        temp[2] = 0x02;
        QspiSendAndGetWords(temp, temp, 3);
        WaitQSPI_Busy();    
        QspiFlashReadStatus();      //wait for operating completely
    }
}


/**================================================================================
            QSPI page write in quad wire mode
  ================================================================================*/
void QspiFlashProgram(uint32_t PrgAddr, uint32_t* pAddr, uint32_t cnt)
{
    uint32_t    i;  
    uint32_t    tx_buf[256+2] = {0};    
    u32 num = 0;
    uint32_t timeout = 0;

    // page write, don't exceed 256 bytes
    if(cnt > 256)   return;

    tx_buf[0] = 0x32;           // cmd
    tx_buf[1] = PrgAddr;        // addr
    for(i=0; i<cnt; i++)
    {
        tx_buf[i+2] = pAddr[i];
    }
    
    // switch to standard mode, write enable
    QspiInit(STANDARD_SPI_FORMAT_SEL, TX_AND_RX, CTRL1_NDF_CNT);
    QspiFlashWriteEnable(0x06);         
    QspiFlashEnableQE();                
    
    // switch to quad mode, write data 
    QspiInit(QUAD_SPI_FORMAT_SEL, TX_ONLY, cnt+2);  
    for (num = 0; num < cnt+2; num++)
    {
        while (GetQspiTxDataBusyStatus())
        {
            if(++timeout >= 2000)
            {
                break;
            }
        }
        QSPI->DAT0 = *(tx_buf+num);
    }
    
    // switch to standard mode 
    QspiInit(STANDARD_SPI_FORMAT_SEL, TX_AND_RX, CTRL1_NDF_CNT);
    QspiFlashReadStatus();      

}


/**================================================================================
        QSPI page write in single wire mode
================================================================================*/
void SingleSPI_Write(uint32_t addr, uint32_t *buf, uint32_t size)
{
    u32 num = 0;
    uint32_t timeout = 0;
    // switch to single wire mode
    QspiInit(STANDARD_SPI_FORMAT_SEL, TX_AND_RX, CTRL1_NDF_CNT);
    
    // write enable
    QspiFlashWriteEnable(0x06);

    for (num = 0; num < size; num++)
    {
        while (GetQspiTxDataBusyStatus())
        {
            if(++timeout >= 2000)
            {
                break;
            }
        }
        QSPI->DAT0 = *buf++;
    }
    WaitQSPI_Busy();
    QspiFlashReadStatus();  // wait for writing completely
}


/**================================================================================
        QSPI read flash data in quad mode 
    In:     addr    
        *buf    the data from third byte is valid
        size    valid data length
================================================================================*/
void QspiFlashRead(uint32_t addr, uint32_t *buf, uint32_t size)
{
    // quad receive, read data
    QspiInit(QUAD_SPI_FORMAT_SEL, RX_ONLY, size);
    
    QspiSendWord(0x6b);     //CMD
    QspiSendWord(addr);     //ADDR
    Delay(0x100);
    GetFifoData(buf, size);
    WaitQSPI_Busy();
}
/**================================================================================
  * @brief  Erase one region of flash. / Flash erase
  * @param  EraseCmd command to select erase unit.
      0x20:erase 4KB sector.
      0x52 erase 32KB block.
      0xD8 erase 64KB block.
  * @param ErsAddr start address of the erase region.
  ================================================================================*/
void QspiFlashErase(uint32_t EraseCmd, uint32_t ErsAddr)
{
    u32 tx_buf[4], rx_buf[256+3];
    
    // switch to standard write mode
    QspiInit(STANDARD_SPI_FORMAT_SEL, TX_AND_RX, CTRL1_NDF_CNT);
    QspiFlashWriteEnable(0x06);
    
    // transmit the erase command
    tx_buf[0] = EraseCmd;
    tx_buf[1] = (ErsAddr & 0xff0000) >> 16;
    tx_buf[2] = (ErsAddr & 0xff00) >> 8;
    tx_buf[3] = ErsAddr & 0xff;
    QspiSendAndGetWords(tx_buf, rx_buf, 4);
    WaitQSPI_Busy();                    
    ClrFifo();
    QspiFlashReadStatus();  
}


/**================================================================================
        chip erase 
 ================================================================================*/
void QspiFlashChipErase(void)
{
    u32 temp;
    
    // write enable
    QspiFlashWriteEnable(0x06);

    // transmit the erase command
    QspiSendWordAndGetWords(0xc7, &temp, 1); // 0x60 or 0xc7
    WaitQSPI_Busy();                        
    ClrFifo();
    QspiFlashReadStatus();  
}




/**================================================================================
        ≤‚ ‘≥Ã–Ú
================================================================================*/
void QSPI_Test(void)
{
    uint32_t        i, temp;
    uint32_t        buf[4];
    static uint8_t  add;
    
    while(1)
    {
        /** prepare for testing data */ 
        for(i=0; i<256; i++)
        {
            QSPI_TxBuf[i] = i + add;
        }
        add++;      

        //=============================================================
        /**---------- erase test ----------*/
        QspiFlashErase(0x20, TEST_ADDR);                
    
        /**---------- QSPI read and test ----------*/
        for(i=0; i<256; i++)    QSPI_RxBuf[i] = 0;      
        QspiFlashRead(TEST_ADDR, QSPI_RxBuf, 256);
        for(i=0; i<256; i++)
        {
            //log_info(" Erase = 0x%x \n", QSPI_RxBuf[i]);
            if(QSPI_RxBuf[i] != 0xff)
            {
                log_info(" Erase Fail\n");
                return;
            }
        }
        log_info(" Erase OK\n");

        //=============================================================
        /**---------- write test ----------*/
        QspiFlashProgram(TEST_ADDR, QSPI_TxBuf, 256);   
    
        /**---------- QSPI read and print ----------*/
        for(i=0; i<256; i++)    QSPI_RxBuf[i] = 0;      
        QspiFlashRead(0x00, QSPI_RxBuf, 256);
        for(i=0; i<256; i++)        
        {
            //log_info(" read = 0x%x \n", QSPI_RxBuf[i]);   
            if(QSPI_RxBuf[i] != (QSPI_TxBuf[i]&0xff))
            {
                log_info(" Write Fail\n");
                return;
            }   
        }
        log_info(" Write OK\n");

        //=============================================================
        /** QUAD SPI read test*/
        log_info("================= \r\n"); 

        for(i=0; i<2; i++)
        {
            QspiFlashRead(TEST_ADDR+4*i, buf, 4);
            temp = (buf[0]<<24) | (buf[1]<<16) | (buf[2]<<8) | buf[3]; //word print
            log_info(" data = 0x%08x \n", temp);    
        }
            
        log_info("================= \r\n");
        for(i=0; i<8; i++)
        {
            QspiFlashRead(TEST_ADDR+i, &temp, 1);
            log_info(" data = 0x%08x \n", temp);      // byte print
        }
        
        //QspiInit(QUAD_SPI_FORMAT_SEL, RX_ONLY, CTRL1_NDF_CNT);  
        QspiSendWord(0xff);                             
        

        m_delay_ms(5000);   
    }
    

}

/**================================================================================
 *      SPI FLASH initial
 ================================================================================*/
void W25Qxx_Init(void)
{
    uint32_t    temp[3] = {0};
        
    QspiInit(STANDARD_SPI_FORMAT_SEL, TX_AND_RX, CTRL1_NDF_CNT);

	// write enable
    QspiFlashWriteEnable(0x06);
	
    // enable QE
    temp[0] = 0x01;
    temp[1] = 0x02;
    temp[2] = 0x02;
    QspiSendAndGetWords(temp, temp, 3);
    WaitQSPI_Busy();    
    QspiFlashReadStatus();      //wait for operating completely
    
    // read status register1
    temp[0] = 0x05;
    temp[1] = 0xff;
    QspiSendAndGetWords(temp, temp, 2);
    WaitQSPI_Busy();
    
    //read status register2
    temp[0] = 0x35;
    temp[1] = 0xff;
    QspiSendAndGetWords(temp, temp, 2);
    WaitQSPI_Busy();
}





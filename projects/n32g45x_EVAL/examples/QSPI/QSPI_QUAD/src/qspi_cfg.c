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
 * @file qspi_cfg.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
#include "qspi_cfg.h"
#include "n32g45x_qspi.h"
#include "n32g45x_dma.h"
#include "n32g45x_gpio.h"

/**
  * @brief  Select format of QSPI.
  * @param qspi_format_sel Select format of QSPI.
                        STANDARD_SPI_FORMAT_SEL:Standard spi
                        DUAL_SPI_FORMAT_SEL:Dual spi
                        QUAD_SPI_FORMAT_SEL:Quad spi
                        XIP_SPI_FORMAT_SEL: Memory mapping mode
  * @param data_dir The direction of transferring data.
                        TX_AND_RX:Transmit and receive data.
                        RX_ONLY:Receive data only.
                        TX_ONLY:Transmit data only.
  * @param count Number of data frames. It is valid only in RX_ONLY mode of DUAL_SPI_FORMAT_SEL or QUAD_SPI_FORMAT_SEL.
  */
void QspiInit(QSPI_FORMAT_SEL qspi_format_sel, QSPI_DATA_DIR data_dir, uint16_t count)
{
    QSPI_InitType QSPI_InitStruct = {0};

    switch (qspi_format_sel)
    {
        case STANDARD_SPI_FORMAT_SEL:
            QSPI_GPIO(QSPI_AFIO_PORT_SEL, 0, 1);

            QSPI_DeInit();
            RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_QSPI, ENABLE);
            QSPI_InitStruct.SPI_FRF     = QSPI_CTRL0_SPI_FRF_STANDARD_FORMAT; //standard SPI frame format
            QSPI_InitStruct.TMOD        = QSPI_CTRL0_TMOD_TX_AND_RX;  //transfer mode
            QSPI_InitStruct.SCPOL       = QSPI_CTRL0_SCPOL_HIGH; //serial clock polarity
            QSPI_InitStruct.SCPH        = QSPI_CTRL0_SCPH_SECOND_EDGE; //serial clock phase
            QSPI_InitStruct.DFS         = QSPI_CTRL0_DFS_8_BIT; //data frame size
            QSPI_InitStruct.CLK_DIV     = CLOCK_DIVIDER;   //clock divider
            QSPI_InitStruct.TXFT        = QSPI_TXFT_TEI_0; //transmit fifo threshold
            QSPI_InitStruct.RXFT        = QSPI_RXFT_TFI_0; //receive fifo threshold
            QSPI_InitStruct.NDF         = CTRL1_NDF_CNT;      //number of data frames

            QspiInitConfig(&QSPI_InitStruct);

            QSPI_Cmd(ENABLE);
            break;
        
        case DUAL_SPI_FORMAT_SEL:
            QSPI_GPIO(QSPI_AFIO_PORT_SEL, 0, 0);

            QSPI_DeInit();
            RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_QSPI, ENABLE);                //enable clock of qspi
            GPIO_ConfigPinRemap(GPIO_RMP_QSPI_XIP_EN, DISABLE);                 //disable XIP
            
            QSPI_InitStruct.SPI_FRF = QSPI_CTRL0_SPI_FRF_DUAL_FORMAT;   //dual SPI frame format
            if (data_dir == TX_ONLY) 
            {
                QSPI_InitStruct.TMOD = QSPI_CTRL0_TMOD_TX_ONLY;    //TX transfer mode
                QSPI_InitStruct.NDF                   = 0;         //number of data frames
                QSPI_InitStruct.ENHANCED_WAIT_CYCLES  = 0;           //wait cycles of dummy
            }
            else if (data_dir == RX_ONLY)
            {
                QSPI_InitStruct.TMOD = QSPI_CTRL0_TMOD_RX_ONLY;  //RX transfer mode
                QSPI_InitStruct.NDF                   = count;     //number of data frames
                QSPI_InitStruct.ENHANCED_WAIT_CYCLES  = QSPI_ENH_CTRL0_WAIT_8CYCLES; //wait cycles of dummy
            }
            QSPI_InitStruct.CFS                  = QSPI_CTRL0_CFS_8_BIT;  //control frame size
            QSPI_InitStruct.SCPOL                = QSPI_CTRL0_SCPOL_HIGH; //serial clock polarity
            QSPI_InitStruct.SCPH                 = QSPI_CTRL0_SCPH_SECOND_EDGE; //serial clock phase
            QSPI_InitStruct.FRF                  = QSPI_CTRL0_FRF_MOTOROLA; //frame format
            QSPI_InitStruct.DFS                  = QSPI_CTRL0_DFS_8_BIT; //data frame size
            QSPI_InitStruct.CLK_DIV              = CLOCK_DIVIDER;  //clock divider
            QSPI_InitStruct.TXFT                 = QSPI_TXFT_TEI_0; //transmit fifo threshold
            QSPI_InitStruct.RXFT                 = QSPI_RXFT_TFI_0; //receive fifo threshold
             
            QSPI_InitStruct.ENHANCED_CLK_STRETCH_EN = QSPI_ENH_CTRL0_CLK_STRETCH_EN; //enable stretch
            QSPI_InitStruct.ENHANCED_ADDR_LEN       = QSPI_ENH_CTRL0_ADDR_LEN_24_BIT; //length of address to transmit
            QSPI_InitStruct.ENHANCED_INST_L         = QSPI_ENH_CTRL0_INST_L_8_LINE; //instruction length
            
            QspiInitConfig(&QSPI_InitStruct);   
            QSPI_Cmd(ENABLE);
            break;
            
        case QUAD_SPI_FORMAT_SEL:
            QSPI_GPIO(QSPI_AFIO_PORT_SEL, 0, 0);

            QSPI_DeInit();
            RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_QSPI, ENABLE);                //enable clock of qspi
            GPIO_ConfigPinRemap(GPIO_RMP_QSPI_XIP_EN, DISABLE);                 //disable XIP
            
            QSPI_InitStruct.SPI_FRF = QSPI_CTRL0_SPI_FRF_QUAD_FORMAT;   //quad SPI frame format
            if (data_dir == TX_ONLY) 
            {
                QSPI_InitStruct.TMOD = QSPI_CTRL0_TMOD_TX_ONLY;    //TX transfer mode
                QSPI_InitStruct.NDF                     = 0;         //number of data frames
                QSPI_InitStruct.ENHANCED_WAIT_CYCLES  = 0;           //wait cycles of dummy
            }
            else if (data_dir == RX_ONLY)
            {
                QSPI_InitStruct.TMOD = QSPI_CTRL0_TMOD_RX_ONLY;  //RX transfer mode
                QSPI_InitStruct.NDF                     = count;     //number of data frames
                QSPI_InitStruct.ENHANCED_WAIT_CYCLES  = QSPI_ENH_CTRL0_WAIT_8CYCLES; //wait cycles of dummy
            }
            QSPI_InitStruct.CFS                  = QSPI_CTRL0_CFS_8_BIT;  //control frame size
            QSPI_InitStruct.SCPOL                = QSPI_CTRL0_SCPOL_HIGH; //serial clock polarity
            QSPI_InitStruct.SCPH                 = QSPI_CTRL0_SCPH_SECOND_EDGE; //serial clock phase
            QSPI_InitStruct.FRF                  = QSPI_CTRL0_FRF_MOTOROLA; //frame format
            QSPI_InitStruct.DFS                  = QSPI_CTRL0_DFS_8_BIT; //data frame size
            QSPI_InitStruct.CLK_DIV              = CLOCK_DIVIDER;  //clock divider
            QSPI_InitStruct.TXFT                 = QSPI_TXFT_TEI_0; //transmit fifo threshold
            QSPI_InitStruct.RXFT                 = QSPI_RXFT_TFI_0; //receive fifo threshold
             
            QSPI_InitStruct.ENHANCED_CLK_STRETCH_EN = QSPI_ENH_CTRL0_CLK_STRETCH_EN; //enable stretch
            QSPI_InitStruct.ENHANCED_ADDR_LEN       = QSPI_ENH_CTRL0_ADDR_LEN_24_BIT; //length of address to transmit
            QSPI_InitStruct.ENHANCED_INST_L         = QSPI_ENH_CTRL0_INST_L_8_LINE; //instruction length
            
            QspiInitConfig(&QSPI_InitStruct);   
            QSPI_Cmd(ENABLE);
            break;
            
        case XIP_SPI_FORMAT_SEL:
            QSPI_GPIO(QSPI_AFIO_PORT_SEL, 0, 0);

            QSPI_DeInit();
            RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_QSPI, ENABLE);                //enable clock of qspi
            GPIO_ConfigPinRemap(GPIO_RMP_QSPI_XIP_EN, DISABLE);                 //disable XIP
            
            QSPI_InitStruct.SPI_FRF = QSPI_CTRL0_SPI_FRF_QUAD_FORMAT;           //quad SPI frame format
            QSPI_InitStruct.TMOD                 = QSPI_CTRL0_TMOD_RX_ONLY;  //transfer mode
            QSPI_InitStruct.NDF                  = count;     //number of data frames
            QSPI_InitStruct.CFS                  = QSPI_CTRL0_CFS_8_BIT;  //control frame size
            QSPI_InitStruct.SCPOL                = QSPI_CTRL0_SCPOL_HIGH; //serial clock polarity
            QSPI_InitStruct.SCPH                 = QSPI_CTRL0_SCPH_SECOND_EDGE; //serial clock phase
            QSPI_InitStruct.FRF                  = QSPI_CTRL0_FRF_MOTOROLA; //frame format
            QSPI_InitStruct.DFS                  = QSPI_CTRL0_DFS_8_BIT; //data frame size
            QSPI_InitStruct.CLK_DIV              = CLOCK_DIVIDER;  //clock divider
            QSPI_InitStruct.TXFT                 = QSPI_TXFT_TEI_0; //transmit fifo threshold
            QSPI_InitStruct.RXFT                 = QSPI_RXFT_TFI_0; //receive fifo threshold
             
            QSPI_InitStruct.ENHANCED_CLK_STRETCH_EN = QSPI_ENH_CTRL0_CLK_STRETCH_EN; //enable stretch

            QSPI_InitStruct.XIP_MBL         = QSPI_XIP_CTRL_XIP_MBL_LEN_8_BIT; //XIP mode bits length
            QSPI_InitStruct.XIP_CT_EN       = QSPI_XIP_CTRL_XIP_CT_EN; //enable continuous transfer in XIP mode
            QSPI_InitStruct.XIP_INST_EN     = QSPI_XIP_CTRL_XIP_INST_EN; //enable XIP instruction
            //QSPI_InitStruct.XIP_DFS_HC    = QSPI_XIP_CTRL_DFS_HC; //Fix DFS for XIP transfer
            QSPI_InitStruct.XIP_ADDR_LEN    = QSPI_XIP_CTRL_ADDR_24BIT; //length of address to transmit
            QSPI_InitStruct.XIP_INST_L      = QSPI_XIP_CTRL_INST_L_8_LINE; //instruction length
            QSPI_InitStruct.XIP_WAIT_CYCLES = QSPI_XIP_CTRL_WAIT_8CYCLES; //wait cycles of dummy
            QSPI_InitStruct.XIP_FRF         = QSPI_XIP_CTRL_FRF_4_LINE; //frame format
            QSPI_InitStruct.XIP_MD_BITS     = 0xaabb;       // content of mode stage
            QSPI_InitStruct.ITOC = 0x6b;        // 0X6B = QUAD Read
            QSPI_InitStruct.WTOC = 0x6b;        // WTOC
            
            QspiInitConfig(&QSPI_InitStruct);
        
            /** enable XIP */
            QSPI_XIP_Cmd(ENABLE);
            QSPI_Cmd(ENABLE);           

            GPIO_ConfigPinRemap(GPIO_RMP_QSPI_XIP_EN, ENABLE);      // enable memory map of XIP mode
            break;
            
        default:
                break;
    }
}


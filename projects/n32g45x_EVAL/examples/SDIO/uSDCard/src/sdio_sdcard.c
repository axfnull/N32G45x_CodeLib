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
 * @file sdio_sdcard.c
 * @author Nations
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include <string.h>
#include "sdio_sdcard.h"

/**
 * @brief  SDIO Static flags, TimeOut, DATFIFO Address
 */
//#define NULL              0
#define SDIO_STATIC_FLAGS ((uint32_t)0x000005FF)
#define SDIO_CMD0TIMEOUT  ((uint32_t)0x00010000)

/**
 * @brief  Mask for errors Card Status R1 (OCR Register)
 */
#define SD_OCR_ADDR_OUT_OF_RANGE     ((uint32_t)0x80000000)
#define SD_OCR_ADDR_MISALIGNED       ((uint32_t)0x40000000)
#define SD_OCR_BLOCK_LEN_ERR         ((uint32_t)0x20000000)
#define SD_OCR_ERASE_SEQ_ERR         ((uint32_t)0x10000000)
#define SD_OCR_BAD_ERASE_PARAM       ((uint32_t)0x08000000)
#define SD_OCR_WRITE_PROT_VIOLATION  ((uint32_t)0x04000000)
#define SD_OCR_LOCK_UNLOCK_FAILED    ((uint32_t)0x01000000)
#define SD_OCR_COM_CRC_FAILED        ((uint32_t)0x00800000)
#define SD_OCR_ILLEGAL_CMD           ((uint32_t)0x00400000)
#define SD_OCR_CARD_ECC_FAILED       ((uint32_t)0x00200000)
#define SD_OCR_CC_ERROR              ((uint32_t)0x00100000)
#define SD_OCR_GENERAL_UNKNOWN_ERROR ((uint32_t)0x00080000)
#define SD_OCR_STREAM_READ_UNDERRUN  ((uint32_t)0x00040000)
#define SD_OCR_STREAM_WRITE_OVERRUN  ((uint32_t)0x00020000)
#define SD_OCR_CID_CSD_OVERWRIETE    ((uint32_t)0x00010000)
#define SD_OCR_WP_ERASE_SKIP         ((uint32_t)0x00008000)
#define SD_OCR_CARD_ECC_DISABLED     ((uint32_t)0x00004000)
#define SD_OCR_ERASE_RESET           ((uint32_t)0x00002000)
#define SD_OCR_AKE_SEQ_ERROR         ((uint32_t)0x00000008)
#define SD_OCR_ERRORBITS             ((uint32_t)0xFDFFE008)

/**
 * @brief  Masks for R6 Response
 */
#define SD_R6_GENERAL_UNKNOWN_ERROR ((uint32_t)0x00002000)
#define SD_R6_ILLEGAL_CMD           ((uint32_t)0x00004000)
#define SD_R6_COM_CRC_FAILED        ((uint32_t)0x00008000)

#define SD_VOLTAGE_WINDOW_SD  ((uint32_t)0x80100000)
#define SD_VOLTAGE_WINDOW_MMC ((uint32_t)0x00FF8000)

#define SD_HIGH_CAPACITY ((uint32_t)0x40000000)
#define SD_STD_CAPACITY  ((uint32_t)0x00000000)
#define SD_CHECK_PATTERN ((uint32_t)0x000001AA)

#define SD_MAX_VOLT_TRIAL ((uint32_t)0x0000FFFF)
#define SD_ALLZERO        ((uint32_t)0x00000000)

#define SD_WIDE_BUS_SUPPORT   ((uint32_t)0x00040000)
#define SD_SINGLE_BUS_SUPPORT ((uint32_t)0x00010000)
#define SD_CARD_LOCKED        ((uint32_t)0x02000000)

#define SD_DATATIMEOUT     ((uint32_t)0xFFFFFFFF)
#define SD_0TO7BITS        ((uint32_t)0x000000FF)
#define SD_8TO15BITS       ((uint32_t)0x0000FF00)
#define SD_16TO23BITS      ((uint32_t)0x00FF0000)
#define SD_24TO31BITS      ((uint32_t)0xFF000000)
#define SD_MAX_DATA_LENGTH ((uint32_t)0x01FFFFFF)

#define SD_HALFFIFO      ((uint32_t)0x00000008)
#define SD_HALFFIFOBYTES ((uint32_t)0x00000020)

/**
 * @brief  Command Class Supported
 */
#define SD_CCCC_LOCK_UNLOCK ((uint32_t)0x00000080)
#define SD_CCCC_WRITE_PROT  ((uint32_t)0x00000040)
#define SD_CCCC_ERASE       ((uint32_t)0x00000020)

/**
 * @brief  Following commands are SD Card Specific commands.
 *         SDIO_APP_CMD should be sent before sending these commands.
 */
#define SDIO_SEND_IF_COND ((uint32_t)0x00000008)
#define SDIO_DATATIMEOUT  ((u32)0xFFFFFFFF)

uint32_t CardType =
    SDIO_STD_CAPACITY_SD_CARD_V1_1;       // The type of memory card. First, initialize it as a 1.1 protocol card
uint32_t CSD_Tab[4], CID_Tab[4], RCA = 0; // SD card CSD, CID and RCA data
static uint8_t SDSTATUS_Tab[16];          // Memory card status, part of CTRLSTS
__IO uint32_t StopCondition = 0;          // Flag to stop card operation
__IO SD_Error TransferError = SD_OK;      // Used for memory card error, initialized to normal state
__IO uint32_t TransferEnd   = 0; // Used to mark whether the transmission ends or not in the interrupt service function.
SD_CardInfo SDCardInfo;          // Information for memory card, part of DSR

/*SDIO initialized structure*/
SDIO_InitType SDIO_InitStructure;
SDIO_CmdInitType SDIO_CmdInitStructure;
SDIO_DataInitType SDIO_DataInitStructure;

static SD_Error CmdError(void);
// static SD_Error CmdResp1Error(uint8_t cmd);
static SD_Error CmdResp7Error(void);
static SD_Error CmdResp3Error(void);
static SD_Error CmdResp2Error(void);
static SD_Error CmdResp6Error(uint8_t cmd, uint16_t* prca);
static SD_Error SDEnWideBus(FunctionalState Cmd);
static SD_Error MMCSetWideBus(u32 WideMode);

static SD_Error IsCardProgramming(uint8_t* pstatus);
static SD_Error FindSCR(uint16_t rca, uint32_t* pscr);
static SD_Error CmdResp1Error_ForIrqTest(uint8_t cmd);

static uint32_t SD_DMAEndOfTransferStatus(void);
static void SD_DMA_RxConfig(uint32_t* BufferDST, uint32_t BufferSize);
static void SD_DMA_TxConfig(uint32_t* BufferSRC, uint32_t BufferSize);

uint8_t convert_from_bytes_to_power_of_two(uint16_t NumberOfBytes);

/*
The special buf of SD readdisk / SD writedisk function.
When the data cache address of these two functions is not 4-byte aligned,
this array is needed to ensure that the data cache address is 4-byte aligned
*/

#ifdef __IAR_ARM
#pragma pack(4) 
u8 SDIO_DATA_BUFFER[512];
#pragma pack() 
#else
__align(4) u8 SDIO_DATA_BUFFER[512];
#endif

/**
 * @brief  SD_DeInit,Reset SDIO port
 *
 */
void SD_DeInit(void)
{
    GPIO_InitType GPIO_InitStructure;

    /*!< Disable SDIO Clock */
    SDIO_EnableClock(DISABLE);

    /*!< Set Power State to OFF */
    SDIO_SetPower(SDIO_POWER_CTRL_OFF);

    /*!< DeInitializes the SDIO peripheral */
    SDIO_DeInit();

    /*!< Disable the SDIO AHB Clock */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_SDIO, DISABLE);

    /*!< Configure PC.08, PC.09, PC.10, PC.11, PC.12 pin: D0, D1, D2, D3, CLK pin */
    GPIO_InitStructure.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);

    /*!< Configure PD.02 CMDCTRL line */
    GPIO_InitStructure.Pin = GPIO_PIN_2;
    GPIO_InitPeripheral(GPIOD, &GPIO_InitStructure);
}

/**
 * @brief  SDIO priority configuration function
 * @param 0 Disable, 1:Enable
 */
void NVIC_SDIO_Configuration(FunctionalState Cmd)
{
    NVIC_InitType NVIC_InitStructure;

    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    NVIC_InitStructure.NVIC_IRQChannel                   = SDIO_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = Cmd;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  Returns the DMA End Of Transfer Status.
 * @return DMA SDIO Channel Status.
 */
uint32_t SD_DMAEndOfTransferStatus(void)
{
    return (uint32_t)DMA_GetFlagStatus(DMA2_FLAG_TC4, DMA2); // Channel4 transfer complete flag.
}

uint32_t SD_DMAEndOfTransferStatus_ch(u32 ch)
{
    u32 DMA2_TC[8] = {DMA2_FLAG_TC1,
                      DMA2_FLAG_TC2,
                      DMA2_FLAG_TC3,
                      DMA2_FLAG_TC4,
                      DMA2_FLAG_TC5,
                      DMA2_FLAG_TC6,
                      DMA2_FLAG_TC7,
                      DMA2_FLAG_TC8};
    return (uint32_t)DMA_GetFlagStatus(DMA2_TC[ch], DMA2); // Channel4 transfer complete flag.
}

/**
 * @brief  DMA RX config
 * @param BufferDST Variable pointer for loading data
 *         BufferSize:  Buffer size
 */
void SD_DMA_RxConfig(uint32_t* BufferDST, uint32_t BufferSize)
{
    DMA_InitType DMA_InitStructure;

    DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4, DMA2); // Clear DMA flag bit

    /*!< DMA2 Channel4 disable */
    DMA_EnableChannel(DMA2_CH4, DISABLE); // SDIO is the fourth channel

    /*!< DMA2 Channel4 Config */
    DMA_InitStructure.PeriphAddr = (uint32_t)SDIO_FIFO_ADDRESS; // Peripheral address,fifo
    DMA_InitStructure.MemAddr    = (uint32_t)BufferDST;         // Destination address
    DMA_InitStructure.Direction  = DMA_DIR_PERIPH_SRC;          // Peripheral as original address
    DMA_InitStructure.BufSize    = BufferSize / 4;              // 1/4 cache size
    DMA_InitStructure.PeriphInc  = DMA_PERIPH_INC_DISABLE; // Enable peripheral address not to increase automatically
    DMA_InitStructure.DMA_MemoryInc = DMA_MEM_INC_ENABLE;  // Enable storage target address auto increment
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_WORD; // Peripheral data size is word, 32-bit
    DMA_InitStructure.MemDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.CircularMode = DMA_MODE_NORMAL; // No loop, loop mode is mainly used on ADC
    DMA_InitStructure.Priority = DMA_PRIORITY_HIGH;   // High channel priority
    DMA_InitStructure.Mem2Mem  = DMA_M2M_DISABLE;     // Non memory to memory mode
    DMA_Init(DMA2_CH4, &DMA_InitStructure);

    /*!< DMA2 Channel4 enable */ // Do not set DMA interrupt
    DMA_EnableChannel(DMA2_CH4, ENABLE);
}

/**
 * @brief  DMA TX config
 * @param BufferSRC Variable pointer for loading data
 *         BufferSize:  Buffer size
 */
void SD_DMA_TxConfig(uint32_t* BufferSRC, uint32_t BufferSize)
{
    DMA_InitType DMA_InitStructure;
    DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4, DMA2);

    /*!< DMA2 Channel4 disable */
    DMA_EnableChannel(DMA2_CH4, DISABLE);

    /*!< DMA2 Channel4 Config */
    DMA_InitStructure.PeriphAddr     = (uint32_t)SDIO_FIFO_ADDRESS;
    DMA_InitStructure.MemAddr        = (uint32_t)BufferSRC;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_DST; // Peripheral as write target
    DMA_InitStructure.BufSize        = BufferSize / 4;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE; // Peripheral address does not increase automatically
    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_WORD;
    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_Word;
    DMA_InitStructure.CircularMode   = DMA_MODE_NORMAL;
    DMA_InitStructure.Priority       = DMA_PRIORITY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;
    DMA_Init(DMA2_CH4, &DMA_InitStructure);

    /*!< DMA2 Channel4 enable */
    DMA_EnableChannel(DMA2_CH4, ENABLE);
}

void SD_DMA_TxConfig_ForDMATest(uint32_t* BufferSRC, uint32_t BufferSize, uint32_t ch)
{
    DMA_ChannelType* DMA2_ch[8] = {DMA2_CH1, DMA2_CH2, DMA2_CH3, DMA2_CH4, DMA2_CH5, DMA2_CH6, DMA2_CH7, DMA2_CH8};
    u32 DMA2_TC[8]              = {DMA2_FLAG_TC1,
                      DMA2_FLAG_TC2,
                      DMA2_FLAG_TC3,
                      DMA2_FLAG_TC4,
                      DMA2_FLAG_TC5,
                      DMA2_FLAG_TC6,
                      DMA2_FLAG_TC7,
                      DMA2_FLAG_TC8};
    u32 DMA2_TE[8]              = {DMA2_FLAG_TE1,
                      DMA2_FLAG_TE2,
                      DMA2_FLAG_TE3,
                      DMA2_FLAG_TE4,
                      DMA2_FLAG_TE5,
                      DMA2_FLAG_TE6,
                      DMA2_FLAG_TE7,
                      DMA2_FLAG_TE8};
    u32 DMA2_HT[8]              = {DMA2_FLAG_HT1,
                      DMA2_FLAG_HT2,
                      DMA2_FLAG_HT3,
                      DMA2_FLAG_HT4,
                      DMA2_FLAG_HT5,
                      DMA2_FLAG_HT6,
                      DMA2_FLAG_HT7,
                      DMA2_FLAG_HT8};
    u32 DMA2_GL[8]              = {DMA2_FLAG_GL1,
                      DMA2_FLAG_GL2,
                      DMA2_FLAG_GL3,
                      DMA2_FLAG_GL4,
                      DMA2_FLAG_GL5,
                      DMA2_FLAG_GL6,
                      DMA2_FLAG_GL7,
                      DMA2_FLAG_GL8};

    DMA_InitType DMA_InitStructure;
    DMA_ClearFlag(DMA2_TC[ch] | DMA2_TE[ch] | DMA2_HT[ch] | DMA2_GL[ch], DMA2);

    /*!< DMA2 Channel4 disable */
    DMA_EnableChannel(DMA2_ch[ch], DISABLE);

    /*!< DMA2 Channel4 Config */
    DMA_InitStructure.PeriphAddr     = (uint32_t)SDIO_FIFO_ADDRESS;
    DMA_InitStructure.MemAddr        = (uint32_t)BufferSRC;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_DST; // Peripheral as write target
    DMA_InitStructure.BufSize        = BufferSize / 4;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE; // Peripheral address does not increase automatically
    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_WORD;
    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_Word;
    DMA_InitStructure.CircularMode   = DMA_MODE_NORMAL;
    DMA_InitStructure.Priority       = DMA_PRIORITY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;
    DMA_Init(DMA2_ch[ch], &DMA_InitStructure);

    /*!< DMA2 Channel4 enable */
    DMA_EnableChannel(DMA2_ch[ch], ENABLE);
}

void SD_DMA_RxConfig_ForDMATest(uint32_t* BufferDST, uint32_t BufferSize, uint32_t ch)
{
    DMA_ChannelType* DMA2_ch[8] = {DMA2_CH1, DMA2_CH2, DMA2_CH3, DMA2_CH4, DMA2_CH5, DMA2_CH6, DMA2_CH7, DMA2_CH8};
    u32 DMA2_TC[8]              = {DMA2_FLAG_TC1,
                      DMA2_FLAG_TC2,
                      DMA2_FLAG_TC3,
                      DMA2_FLAG_TC4,
                      DMA2_FLAG_TC5,
                      DMA2_FLAG_TC6,
                      DMA2_FLAG_TC7,
                      DMA2_FLAG_TC8};
    u32 DMA2_TE[8]              = {DMA2_FLAG_TE1,
                      DMA2_FLAG_TE2,
                      DMA2_FLAG_TE3,
                      DMA2_FLAG_TE4,
                      DMA2_FLAG_TE5,
                      DMA2_FLAG_TE6,
                      DMA2_FLAG_TE7,
                      DMA2_FLAG_TE8};
    u32 DMA2_HT[8]              = {DMA2_FLAG_HT1,
                      DMA2_FLAG_HT2,
                      DMA2_FLAG_HT3,
                      DMA2_FLAG_HT4,
                      DMA2_FLAG_HT5,
                      DMA2_FLAG_HT6,
                      DMA2_FLAG_HT7,
                      DMA2_FLAG_HT8};
    u32 DMA2_GL[8]              = {DMA2_FLAG_GL1,
                      DMA2_FLAG_GL2,
                      DMA2_FLAG_GL3,
                      DMA2_FLAG_GL4,
                      DMA2_FLAG_GL5,
                      DMA2_FLAG_GL6,
                      DMA2_FLAG_GL7,
                      DMA2_FLAG_GL8};

    DMA_InitType DMA_InitStructure;
    DMA_ClearFlag(DMA2_TC[ch] | DMA2_TE[ch] | DMA2_HT[ch] | DMA2_GL[ch], DMA2); // Clear DMA flag bit

    /*!< DMA2 Channel4 disable */
    DMA_EnableChannel(DMA2_ch[ch], DISABLE); // SDIO is the fourth channel

    /*!< DMA2 Channel4 Config */
    DMA_InitStructure.PeriphAddr = (uint32_t)SDIO_FIFO_ADDRESS; // Peripheral address,fifo
    DMA_InitStructure.MemAddr    = (uint32_t)BufferDST;         // Destination address
    DMA_InitStructure.Direction  = DMA_DIR_PERIPH_SRC;          // Peripheral as source address
    DMA_InitStructure.BufSize    = BufferSize / 4;              // 1 / 4 cache size
    DMA_InitStructure.PeriphInc  = DMA_PERIPH_INC_DISABLE; // Enable peripheral address not to increase automatically
    DMA_InitStructure.DMA_MemoryInc = DMA_MEM_INC_ENABLE;  // Enable storage target address auto increment
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_WORD; // Peripheral data size is word, 32-bit
    DMA_InitStructure.MemDataSize = DMA_MemoryDataSize_Word;      // Peripheral data size is word, 32-bit
    DMA_InitStructure.CircularMode = DMA_MODE_NORMAL;             // No loop, loop mode is mainly used on ADC
    DMA_InitStructure.Priority = DMA_PRIORITY_HIGH;               // High channel priority
    DMA_InitStructure.Mem2Mem  = DMA_M2M_DISABLE;                 // Non memory to memory mode
    DMA_Init(DMA2_ch[ch], &DMA_InitStructure);

    /*!< DMA2 Channel4 enable */ // Do not set DMA interrupt
    DMA_EnableChannel(DMA2_ch[ch], ENABLE);
}

/**
 * @brief  Initialize the SDIO pins and turn on the clock
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;
    // Initial PE0?PE1 Card Detect function
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOE, ENABLE);
    /*!< Configure PE.0, PE.1*/
    GPIO_InitStructure.Pin        = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitPeripheral(GPIOE, &GPIO_InitStructure);

#if 1
    // if(GPIO_ReadInputDataBit(GPIOE,  GPIO_PIN_1))  //PE1   TF card
    {
        /*!< GPIOC and GPIOD Periph clock enable */
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_GPIOC | RCC_APB2_PERIPH_GPIOD, ENABLE);

        /*!< Configure PC.08, PC.09, PC.10, PC.11, PC.12 pin: D0, D1, D2, D3, CLK pin D6, D7 */
        GPIO_InitStructure.Pin =
            GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);

        /*!< Configure PD.02 CMDCTRL line */
        GPIO_InitStructure.Pin = GPIO_PIN_2;
        GPIO_InitPeripheral(GPIOD, &GPIO_InitStructure);

        /*!< Configure PB.08  9   D4  D5 */
        GPIO_InitStructure.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
    }
#else
    // if(GPIO_ReadInputDataBit(GPIOE,  GPIO_PIN_0))  //PE0   eMMC card
    {
        RCC_EnableAPB2PeriphClk(
            RCC_APB2_PERIPH_AFIO | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_GPIOC | RCC_APB2_PERIPH_GPIOE, ENABLE);
        // AFIO_MAPR3 SDIO_remap
        AFIO->MAPR3 |= 0x1;
        /*!< GPIOE Periph clock has already been enable */

        /*!< Configure PE.08, PE.09, PE.10, PE.11, PE.12, PE.13 pin: D0, D1, D2, D3, CLK pin, CMDCTRL pin */
        GPIO_InitStructure.Pin        = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_InitPeripheral(GPIOE, &GPIO_InitStructure);

        /*!< Configure PB.08    9   D4  D5 */
        GPIO_InitStructure.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

        /*!< Configure PB.08    9   D4  D5 */
        GPIO_InitStructure.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
    }
#endif

    /*!< Enable the SDIO AHB Clock */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_SDIO, ENABLE);

    /*!< Enable the DMA2 Clock */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_DMA2, ENABLE);
}

static void Delay_sdio_lock(u32 count)
{
    volatile u32 j;
    u32 i;
    for (i = 0; i < count; i++)
    {
        j = i;
    }
}

/**
 * @brief  Initialize SD card to make it ready (ready to transfer data)
 * @param ClockBypass if bypass clock
 *         TransClkDiv: Transmission clock division factor
 *         BusWidth   : Transmission line width
 * @return SD_Error: SD card error code,SD_OK if successful
 */
SD_Error SD_Init(u32 ClockBypass, u32 TransClkDiv, u32 BusWidth)
{
    u8 lock_pw[16] = {0};
    u32 resp1      = 0;

    /*Reset SD error status*/
    SD_Error errorstatus = SD_OK;

    /*SDIO peripheral GPIO pin initialization */
    GPIO_Configuration();

    /*Reset all registers of SDIO*/
    SDIO_DeInit();
    /*Power on and carry out card identification process to confirm the operation voltage of the card*/
    errorstatus = SD_PowerON();
    /*If the power is on, the recognition is not successful, and the "response timeout" error is returned*/
    if (errorstatus != SD_OK)
    {
        /*!< CMDCTRL Response TimeOut (wait for CMDSENT flag) */
        return (errorstatus);
    }

    /*Card identification successful, card initialization*/
    errorstatus = SD_InitializeCards();

    if (errorstatus != SD_OK) // Failure to return
    {
        /*!< CMDCTRL Response TimeOut (wait for CMDSENT flag) */
        return (errorstatus);
    }

    /*!< Configure the SDIO peripheral
    After power on identification and card initialization,
          enter data transmission mode to improve reading and writing speed
    If the speed exceeds 24m, enter bypass mode
    !< on N32G45X devices, SDIOCLK is fixed to 48MHz
    !< SDIOCLK = HCLK, SDIO_CK = HCLK/(2 + SDIO_TRANSFER_CLK_DIV) */

    SDIO_InitStructure.ClkDiv  = TransClkDiv;         // SDIO_TRANSFER_CLK_DIV;
    SDIO_InitStructure.ClkEdge = SDIO_CLKEDGE_RISING; // Rising edge acquisition data
    if (ClockBypass)
    {
        SDIO_InitStructure.ClkBypass = SDIO_ClkBYPASS_ENABLE;
    }
    else
    {
        SDIO_InitStructure.ClkBypass = SDIO_ClkBYPASS_DISABLE;
    }

    SDIO_InitStructure.ClkPwrSave       = SDIO_CLKPOWERSAVE_DISABLE; // If this function is enabled, turn off the SD CLK clock when the bus is idle
    SDIO_InitStructure.BusWidth         = SDIO_BUSWIDTH_1B;          // 1 bit mode
    SDIO_InitStructure.HardwareClkCtrl  = SDIO_HARDWARE_CLKCTRL_DISABLE; // Hardware flow. If it is enabled, data transmission will be suspended when
                                                                         // DATFIFO fails to send and receive data
    SDIO_Init(&SDIO_InitStructure);

#if 1
    if (errorstatus == SD_OK)
    {
        /*----------------- Read CSD/CID MSD registers ------------------*/
        errorstatus = SD_GetCardInfo(&SDCardInfo); // Read CSD / CID register
    }

    if (errorstatus == SD_OK)
    {
        /*----------------- Select Card --------------------------------*/
        errorstatus = SD_SelectDeselect((uint32_t)(SDCardInfo.RCA << 16)); // RCA selects the card through cmd7
    }
#endif

    // Check whether the card has been locked and unlocked
    if (SDIO_GetResp(SDIO_RESPONSE_1) & SD_CARD_LOCKED) // Check whether the card has been locked
    {
        lock_pw[0] = 0x5; // SET PW and LOCK
        lock_pw[1] = 14;

        lock_pw[2] = 0x1;
        lock_pw[3] = 0x2;
        lock_pw[4] = 0x3;
        lock_pw[5] = 0x4;
        lock_pw[6] = 0x5;
        lock_pw[7] = 0x6;
        lock_pw[8] = 0x7;
        lock_pw[9] = 0x8;

        lock_pw[10] = 0x1;
        lock_pw[11] = 0x2;
        lock_pw[12] = 0x3;
        lock_pw[13] = 0x4;
        lock_pw[14] = 0x5;
        lock_pw[15] = 0x6;

#if 1
        lock_pw[0] = 0x2; // UNLOCK
        if (SD_WriteBlock_ForLockUnlock(lock_pw, 16) != SD_OK)
        {
            return SD_LOCK_UNLOCK_FAILED;
        }
        while (SDIO->FIFOCOUNT)
        {
            Delay_sdio_lock(0xfff);
        }

        while (resp1 & 0x2000000)
        {
            if (SD_SendStatus(&resp1) != SD_OK)
            {
                return SD_LOCK_UNLOCK_FAILED;
            }

            Delay_sdio_lock(0xfff);
        }
#endif
    }

    if (errorstatus == SD_OK)
    {
        if (SDIO_MULTIMEDIA_CARD == CardType)
        {
            if (BusWidth == 4)
            {
                errorstatus = SD_EnableWideBusOperation(SDIO_BUSWIDTH_4B);
            }
            else if (BusWidth == 8)
            {
                errorstatus = SD_EnableWideBusOperation(SDIO_BUSWIDTH_8B);
            }
            else
            {
                errorstatus = SD_EnableWideBusOperation(SDIO_BUSWIDTH_1B);
            }
        }
        else
        {
            if (BusWidth >= 4)
            {
                errorstatus = SD_EnableWideBusOperation(SDIO_BUSWIDTH_4B);
            }
            else
            {
                errorstatus = SD_EnableWideBusOperation(SDIO_BUSWIDTH_1B);
            }
        }
    }

    NVIC_SDIO_Configuration(ENABLE);
    return (errorstatus);
}

/**
 * @brief  Gets the cuurent sd card data transfer status.
 * @return SDTransferState: Data Transfer state.
 *   This value can be:
 *        - SD_TRANSFER_OK: No data transfer is acting
 *        - SD_TRANSFER_BUSY: Data transfer is acting
 */
SDTransferState SD_GetStatus(void)
{
    SDCardState cardstate = SD_CARD_TRANSFER;

    cardstate = SD_GetState();

    if (cardstate == SD_CARD_TRANSFER)
    {
        return (SD_TRANSFER_OK);
    }
    else if (cardstate == SD_CARD_ERROR)
    {
        return (SD_TRANSFER_ERROR);
    }
    else
    {
        return (SD_TRANSFER_BUSY);
    }
}

/**
 * @brief  Returns the current card's state.
 * @return SDCardState: SD Card Error or SD Card Current State.
 */
SDCardState SD_GetState(void)
{
    uint32_t resp1 = 0;

    if (SD_SendStatus(&resp1) != SD_OK)
    {
        return SD_CARD_ERROR;
    }
    else
    {
        return (SDCardState)((resp1 >> 9) & 0x0F);
    }
}

/**
 * @brief  Open SD card's working voltage and configuration control clock
 * @return SDCardState: SD Card Error or SD Card Current State.
 */
SD_Error SD_PowerON(void)
{
    SD_Error errorstatus = SD_OK;
    uint32_t response = 0, count = 0, validvoltage = 0;
    uint32_t SDType = SD_STD_CAPACITY;

    /*!< Power ON Sequence -----------------------------------------------------*/
    /*!< Configure the SDIO peripheral */
    /*!< SDIOCLK = HCLK, SDIO_CK = HCLK/(2 + SDIO_INIT_CLK_DIV) */
    /*!< on N32G45X devices, SDIOCLK is fixed to 48MHz */
    /*!< SDIO_CK for initialization should not exceed 400 KHz */
    /*The clock at initialization cannot be greater than 400kHz*/
    /*8M:18   18M:45 */

    SDIO_InitStructure.ClkDiv = 178; // SDIO_INIT_CLK_DIV;  /* HCLK = 72MHz, SDIOCLK = 72MHz, SDIO_CK = HCLK/(178 + 2) = 400 KHz */
    SDIO_InitStructure.ClkEdge = SDIO_CLKEDGE_RISING;
    SDIO_InitStructure.ClkBypass = SDIO_ClkBYPASS_DISABLE; // Do not use bypass mode, directly use hclk to divide the frequency to get SDIO "CK"
    SDIO_InitStructure.ClkPwrSave = SDIO_CLKPOWERSAVE_DISABLE;          // Do not turn off clock power when idle
    SDIO_InitStructure.BusWidth = SDIO_BUSWIDTH_1B;                     // 1-bit data line
    SDIO_InitStructure.HardwareClkCtrl = SDIO_HARDWARE_CLKCTRL_DISABLE; // Do not enable hardware flow
    SDIO_Init(&SDIO_InitStructure);

    /*!< Set Power State to ON */
    SDIO_SetPower(SDIO_POWER_CTRL_ON);

    /*!< Enable SDIO Clock */
    SDIO_EnableClock(ENABLE);

    /*Next, send a series of commands to start the card identification process*/
    /*!< CMD0: GO_IDLE_STATE ---------------------------------------------------*/
    /*!< No CMDCTRL response required */
    SDIO_CmdInitStructure.CmdArgument  = 0x0;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_GO_IDLE_STATE; // cmd0
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_NO;         // No response
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig = SDIO_CPSM_ENABLE; // Then CPSM waits for the data transmission to end before starting to send the command.
    SDIO_SendCmd(&SDIO_CmdInitStructure); // Write command into command register

    errorstatus = CmdError(); // Check if cmd0 is received correctly

    if (errorstatus != SD_OK) // Command sending error, return
    {
        /*!< CMDCTRL Response TimeOut (wait for CMDSENT flag) */
        return (errorstatus);
    }

    /*!< CMD8: SEND_IF_COND ----------------------------------------------------*/
    /*!< Send CMD8 to verify SD card interface operating condition */
    /*!< Argument: - [31:12]: Reserved (shall be set to '0')
                 - [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
                 - [7:0]: Check Pattern (recommended 0xAA) */
    /*!< CMDCTRL Response: R7 */
    SDIO_CmdInitStructure.CmdArgument  = SD_CHECK_PATTERN;  // SD will return this parameter after receiving the command
    SDIO_CmdInitStructure.CmdIndex     = SDIO_SEND_IF_COND; // cmd8
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;   // r7
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;      // Turn off wait interrupt
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    /*Check if command received*/
    errorstatus = CmdResp7Error();

    if (errorstatus == SD_OK) // If there is a response, card follows SD protocol version 2.0
    {
        CardType = SDIO_STD_CAPACITY_SD_CARD_V2_0; /*SD card 2.0, first define it as a card of SDSC type*/
        SDType   = SD_HIGH_CAPACITY; // This variable is used as a parameter of acmd41 to ask whether it is an SDSC card
                                   // or an SDHC card
    }
    else // No response, indicates 1.X or MMC card
    {
        /*!< CMD55 */
        SDIO_CmdInitStructure.CmdArgument  = 0x00;
        SDIO_CmdInitStructure.CmdIndex     = SD_CMD_APP_CMD;
        SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
        SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
        SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
        SDIO_SendCmd(&SDIO_CmdInitStructure);
        errorstatus = CmdResp1Error(SD_CMD_APP_CMD);
    }

    // Send cmd55 to detect SD card, MMC card, or unsupported card
    SDIO_CmdInitStructure.CmdArgument  = 0x00;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_APP_CMD;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);
    errorstatus = CmdResp1Error(SD_CMD_APP_CMD); // Whether to respond, MMC or unsupported card is not responding

    /*!< If errorstatus is Command TimeOut, it is a MMC card */
    /*!< If errorstatus is SD_OK it is a SD card: SD card 2.0 (voltage range mismatch)
       or SD card 1.x */
    if (errorstatus == SD_OK) // Responded to cmd55, SD card, may be 1.X, may be 2.0
    {
        /*Next, start to send the voltage range supported by SDIO circularly for a certain number of times*/

        /*!< SD CARD */
        /*!< Send ACMD41 SD_APP_OP_COND with Argument 0x80100000 */
        while ((!validvoltage) && (count < SD_MAX_VOLT_TRIAL))
        {
            // Because acmd41 is used below. It is ACMD command. Before sending ACMD command, send cmd55 to card first
            /*!< SEND CMD55 APP_CMD with RCA as 0 */
            SDIO_CmdInitStructure.CmdArgument  = 0x00;
            SDIO_CmdInitStructure.CmdIndex     = SD_CMD_APP_CMD; // CMD55
            SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
            SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
            SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
            SDIO_SendCmd(&SDIO_CmdInitStructure);

            errorstatus = CmdResp1Error(SD_CMD_APP_CMD); // Detection response

            if (errorstatus != SD_OK)
            {
                return (errorstatus); // No response to cmd55, return
            }

            // acmd41,The command parameters consist of the supported voltage range and HCs bits. The HCS position is
            // used to distinguish whether the card is SDSC or SDHC
            SDIO_CmdInitStructure.CmdArgument  = SD_VOLTAGE_WINDOW_SD | SDType; // The parameters are the voltage range and HCs bit of the host
            SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SD_APP_OP_COND;
            SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r3
            SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
            SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
            SDIO_SendCmd(&SDIO_CmdInitStructure);

            errorstatus = CmdResp3Error(); // Check whether data is received correctly
            if (errorstatus != SD_OK)
            {
                return (errorstatus); // Acmd41 not received correctly, error, return
            }

            /*If the card demand voltage is within the supply voltage range of SDIO, it will automatically power on and
             * mark PWR UUP bit*/
            response = SDIO_GetResp(SDIO_RESPONSE_1); // Read card register, card status
            validvoltage = (((response >> 31) == 1) ? 1 : 0); // Read the PWR up bit of the card's OCR register to see if it is working at normal voltage
            count++;       // Calculate number of cycles
        }
        if (count >= SD_MAX_VOLT_TRIAL) // Cycle detection is over a certain number of times and the power is not on
        {
            errorstatus = SD_INVALID_VOLTRANGE; // SDIO does not support card supply voltage
            return (errorstatus);
        }
        /*check HCS bit in card return information*/
        if (response &= SD_HIGH_CAPACITY) // Judge the CCS bit in OCR. If it is an SDSC card, the following statement
                                          // will not be executed
        {
            CardType = SDIO_HIGH_CAPACITY_SD_CARD; // Change card type from initial SDSC type to SDHC type
        }

    }    /*!< else MMC Card */
    else // MMC Card CMD1 get OP
    {
        while ((!validvoltage) && (count < SD_MAX_VOLT_TRIAL))
        {
            // cmd1,Command parameters consist of supported voltage range bits
            SDIO_CmdInitStructure.CmdArgument  = SD_VOLTAGE_WINDOW_MMC;
            SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SEND_OP_COND;
            SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r3
            SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
            SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
            SDIO_SendCmd(&SDIO_CmdInitStructure);

            errorstatus = CmdResp3Error(); // Check whether data is received correctly
            if (errorstatus != SD_OK)
            {
                return (errorstatus); // CMD1 not received correctly, error, return
            }

            /*If the card demand voltage is within the supply voltage range of SDIO, it will automatically power on and
             * mark PWR UUP bit*/
            response = SDIO_GetResp(SDIO_RESPONSE_1); // Read card register, card status
            validvoltage = (((response >> 31) == 1) ? 1 : 0); // Read the PWR up bit of the card's OCR register to see if it is working at normal voltage
            count++;       // Calculate number of cycles
        }

        if (count >= SD_MAX_VOLT_TRIAL) // Cycle detection is over a certain number of times and the power is not on
        {
            errorstatus = SD_INVALID_VOLTRANGE; // SDIO does not support card supply voltage
            return (errorstatus);
        }

        CardType = SDIO_MULTIMEDIA_CARD;
    }

    return (errorstatus);
}

/**
 * @brief  Turn off the output signal of SDIO
 * @return SDCardState: SD Card Error or SD Card Current State.
 */
SD_Error SD_PowerOFF(void)
{
    SD_Error errorstatus = SD_OK;

    /*!< Set Power State to OFF */
    SDIO_SetPower(SDIO_POWER_CTRL_OFF);

    return (errorstatus);
}

/**
 * @brief  Initialize all cards or single card to ready state
 * @return SDCardState: SD Card Error or SD Card Current State.
 */
SD_Error SD_InitializeCards(void)
{
    SD_Error errorstatus = SD_OK;
    uint16_t rca         = 0x01;

    if (SDIO_GetPower() == SDIO_POWER_CTRL_OFF)
    {
        errorstatus = SD_REQUEST_NOT_APPLICABLE;
        return (errorstatus);
    }

    if (SDIO_SECURE_DIGITAL_IO_CARD != CardType) // judgment card Type
    {
        /*!< Send CMD2 ALL_SEND_CID */
        SDIO_CmdInitStructure.CmdArgument  = 0x0;
        SDIO_CmdInitStructure.CmdIndex     = SD_CMD_ALL_SEND_CID;
        SDIO_CmdInitStructure.ResponseType = SDIO_RESP_LONG;
        SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
        SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
        SDIO_SendCmd(&SDIO_CmdInitStructure);

        errorstatus = CmdResp2Error();

        if (SD_OK != errorstatus)
        {
            return (errorstatus);
        }

        CID_Tab[0] = SDIO_GetResp(SDIO_RESPONSE_1);
        CID_Tab[1] = SDIO_GetResp(SDIO_RESPONSE_2);
        CID_Tab[2] = SDIO_GetResp(SDIO_RESPONSE_3);
        CID_Tab[3] = SDIO_GetResp(SDIO_RESPONSE_4);
    }

    /*Start the SD card initialization process*/
    if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType)
        || (SDIO_SECURE_DIGITAL_IO_COMBO_CARD == CardType)
        || (SDIO_HIGH_CAPACITY_SD_CARD == CardType)) // Using a 2.0 card
    {
        /*!< Send CMD3 SET_REL_ADDR with argument 0 */
        /*!< SD Card publishes its RCA. */
        SDIO_CmdInitStructure.CmdArgument  = 0x00;
        SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_REL_ADDR; // cmd3
        SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;     // r6
        SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
        SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
        SDIO_SendCmd(&SDIO_CmdInitStructure);

        errorstatus = CmdResp6Error(SD_CMD_SET_REL_ADDR, &rca); // Save the relative address of the received card

        if (SD_OK != errorstatus)
        {
            return (errorstatus);
        }
    }

    /*EMMC*/
    if (SDIO_MULTIMEDIA_CARD == CardType)
    {
        /*!< Send CMD3 SET_REL_ADDR with argument 5a */
        /*!< MMC Card accept the RCA. */
        SDIO_CmdInitStructure.CmdArgument  = 0x5a << 16;
        SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_REL_ADDR; // cmd3
        SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;     // r1
        SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
        SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
        SDIO_SendCmd(&SDIO_CmdInitStructure);
        errorstatus = CmdResp1Error(SD_CMD_SET_REL_ADDR);

        if (rca != 0x5a)
        {
            rca = 0x5a;
        }

        if (SD_OK != errorstatus)
        {
            return (errorstatus);
        }
    }

    if (SDIO_SECURE_DIGITAL_IO_CARD != CardType)
    {
        RCA = rca;

        /*!< Send CMD9 SEND_CSD with argument as card's RCA */
        SDIO_CmdInitStructure.CmdArgument  = (uint32_t)(rca << 16);
        SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SEND_CSD;
        SDIO_CmdInitStructure.ResponseType = SDIO_RESP_LONG;
        SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
        SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
        SDIO_SendCmd(&SDIO_CmdInitStructure);

        errorstatus = CmdResp2Error();

        if (SD_OK != errorstatus)
        {
            return (errorstatus);
        }

        CSD_Tab[0] = SDIO_GetResp(SDIO_RESPONSE_1);
        CSD_Tab[1] = SDIO_GetResp(SDIO_RESPONSE_2);
        CSD_Tab[2] = SDIO_GetResp(SDIO_RESPONSE_3);
        CSD_Tab[3] = SDIO_GetResp(SDIO_RESPONSE_4);

        // test CMD10
        SDIO_CmdInitStructure.CmdArgument  = (uint32_t)(rca << 16);
        SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SEND_CID;
        SDIO_CmdInitStructure.ResponseType = SDIO_RESP_LONG;
        SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
        SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
        SDIO_SendCmd(&SDIO_CmdInitStructure);

        errorstatus = CmdResp2Error();

        if (SD_OK != errorstatus)
        {
            return (errorstatus);
        }

        if (CID_Tab[0] != SDIO_GetResp(SDIO_RESPONSE_1) || CID_Tab[1] != SDIO_GetResp(SDIO_RESPONSE_2)
            || CID_Tab[2] != SDIO_GetResp(SDIO_RESPONSE_3) || CID_Tab[3] != SDIO_GetResp(SDIO_RESPONSE_4))
        {
            return (SD_ERROR);
        }
    }
    errorstatus = SD_OK; /*!< All cards get intialized */
    return (errorstatus);
}

/**
 * @brief  Get SD card information
 * @param cardinfo The pointer to the SD_cardinfo structure contains the specific information of the SD card
 * @return SDCardState: SD Card Error or SD Card Current State.
 */
SD_Error SD_GetCardInfo(SD_CardInfo* cardinfo)
{
    SD_Error errorstatus = SD_OK;
    uint8_t tmp          = 0;

    cardinfo->CardType = (uint8_t)CardType;
    cardinfo->RCA      = (uint16_t)RCA;

    /*!< Byte 0 */
    tmp                             = (uint8_t)((CSD_Tab[0] & 0xFF000000) >> 24);
    cardinfo->SD_csd.CSDStruct      = (tmp & 0xC0) >> 6;
    cardinfo->SD_csd.SysSpecVersion = (tmp & 0x3C) >> 2;
    cardinfo->SD_csd.Reserved1      = tmp & 0x03;

    /*!< Byte 1 */
    tmp                   = (uint8_t)((CSD_Tab[0] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.TAAC = tmp;

    /*!< Byte 2 */
    tmp                   = (uint8_t)((CSD_Tab[0] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.NSAC = tmp;

    /*!< Byte 3 */
    tmp                            = (uint8_t)(CSD_Tab[0] & 0x000000FF);
    cardinfo->SD_csd.MaxBusClkFrec = tmp;

    /*!< Byte 4 */
    tmp                              = (uint8_t)((CSD_Tab[1] & 0xFF000000) >> 24);
    cardinfo->SD_csd.CardComdClasses = tmp << 4;

    /*!< Byte 5 */
    tmp = (uint8_t)((CSD_Tab[1] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.CardComdClasses |= (tmp & 0xF0) >> 4;
    cardinfo->SD_csd.RdBlockLen = tmp & 0x0F;

    /*!< Byte 6 */
    tmp                              = (uint8_t)((CSD_Tab[1] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.PartBlockRead   = (tmp & 0x80) >> 7;
    cardinfo->SD_csd.WrBlockMisalign = (tmp & 0x40) >> 6;
    cardinfo->SD_csd.RdBlockMisalign = (tmp & 0x20) >> 5;
    cardinfo->SD_csd.DSRImpl         = (tmp & 0x10) >> 4;
    cardinfo->SD_csd.Reserved2       = 0; /*!< Reserved */

    if ((CardType == SDIO_STD_CAPACITY_SD_CARD_V1_1) || (CardType == SDIO_STD_CAPACITY_SD_CARD_V2_0))
    {
        cardinfo->SD_csd.DeviceSize = (tmp & 0x03) << 10;

        /*!< Byte 7 */
        tmp = (uint8_t)(CSD_Tab[1] & 0x000000FF);
        cardinfo->SD_csd.DeviceSize |= (tmp) << 2;

        /*!< Byte 8 */
        tmp = (uint8_t)((CSD_Tab[2] & 0xFF000000) >> 24);
        cardinfo->SD_csd.DeviceSize |= (tmp & 0xC0) >> 6;

        cardinfo->SD_csd.MaxRdCurrentVDDMin = (tmp & 0x38) >> 3;
        cardinfo->SD_csd.MaxRdCurrentVDDMax = (tmp & 0x07);

        /*!< Byte 9 */
        tmp                                 = (uint8_t)((CSD_Tab[2] & 0x00FF0000) >> 16);
        cardinfo->SD_csd.MaxWrCurrentVDDMin = (tmp & 0xE0) >> 5;
        cardinfo->SD_csd.MaxWrCurrentVDDMax = (tmp & 0x1C) >> 2;
        cardinfo->SD_csd.DeviceSizeMul      = (tmp & 0x03) << 1;
        /*!< Byte 10 */
        tmp = (uint8_t)((CSD_Tab[2] & 0x0000FF00) >> 8);
        cardinfo->SD_csd.DeviceSizeMul |= (tmp & 0x80) >> 7;

        cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1);
        cardinfo->CardCapacity *= (1 << (cardinfo->SD_csd.DeviceSizeMul + 2));
        cardinfo->CardBlockSize = 1 << (cardinfo->SD_csd.RdBlockLen);
        cardinfo->CardCapacity *= cardinfo->CardBlockSize;
    }
    else if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        /*!< Byte 7 */
        tmp                         = (uint8_t)(CSD_Tab[1] & 0x000000FF);
        cardinfo->SD_csd.DeviceSize = (tmp & 0x3F) << 16;

        /*!< Byte 8 */
        tmp = (uint8_t)((CSD_Tab[2] & 0xFF000000) >> 24);

        cardinfo->SD_csd.DeviceSize |= (tmp << 8);

        /*!< Byte 9 */
        tmp = (uint8_t)((CSD_Tab[2] & 0x00FF0000) >> 16);

        cardinfo->SD_csd.DeviceSize |= (tmp);

        /*!< Byte 10 */
        tmp = (uint8_t)((CSD_Tab[2] & 0x0000FF00) >> 8);

        cardinfo->CardCapacity  = (cardinfo->SD_csd.DeviceSize + 1) * 512 * 1024;
        cardinfo->CardBlockSize = 512;
    }

    cardinfo->SD_csd.EraseGrSize = (tmp & 0x40) >> 6;
    cardinfo->SD_csd.EraseGrMul  = (tmp & 0x3F) << 1;

    /*!< Byte 11 */
    tmp = (uint8_t)(CSD_Tab[2] & 0x000000FF);
    cardinfo->SD_csd.EraseGrMul |= (tmp & 0x80) >> 7;
    cardinfo->SD_csd.WrProtectGrSize = (tmp & 0x7F);

    /*!< Byte 12 */
    tmp                                = (uint8_t)((CSD_Tab[3] & 0xFF000000) >> 24);
    cardinfo->SD_csd.WrProtectGrEnable = (tmp & 0x80) >> 7;
    cardinfo->SD_csd.ManDeflECC        = (tmp & 0x60) >> 5;
    cardinfo->SD_csd.WrSpeedFact       = (tmp & 0x1C) >> 2;
    cardinfo->SD_csd.MaxWrBlockLen     = (tmp & 0x03) << 2;

    /*!< Byte 13 */
    tmp = (uint8_t)((CSD_Tab[3] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.MaxWrBlockLen |= (tmp & 0xC0) >> 6;
    cardinfo->SD_csd.WriteBlockPaPartial = (tmp & 0x20) >> 5;
    cardinfo->SD_csd.Reserved3           = 0;
    cardinfo->SD_csd.ContentProtectAppli = (tmp & 0x01);

    /*!< Byte 14 */
    tmp                               = (uint8_t)((CSD_Tab[3] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.FileFormatGrouop = (tmp & 0x80) >> 7;
    cardinfo->SD_csd.CopyFlag         = (tmp & 0x40) >> 6;
    cardinfo->SD_csd.PermWrProtect    = (tmp & 0x20) >> 5;
    cardinfo->SD_csd.TempWrProtect    = (tmp & 0x10) >> 4;
    cardinfo->SD_csd.FileFormat       = (tmp & 0x0C) >> 2;
    cardinfo->SD_csd.ECC              = (tmp & 0x03);

    /*!< Byte 15 */
    tmp                        = (uint8_t)(CSD_Tab[3] & 0x000000FF);
    cardinfo->SD_csd.CSD_CRC   = (tmp & 0xFE) >> 1;
    cardinfo->SD_csd.Reserved4 = 1;

    /*!< Byte 0 */
    tmp                             = (uint8_t)((CID_Tab[0] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ManufacturerID = tmp;

    /*!< Byte 1 */
    tmp                          = (uint8_t)((CID_Tab[0] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.OEM_AppliID = tmp << 8;

    /*!< Byte 2 */
    tmp = (uint8_t)((CID_Tab[0] & 0x000000FF00) >> 8);
    cardinfo->SD_cid.OEM_AppliID |= tmp;

    /*!< Byte 3 */
    tmp                        = (uint8_t)(CID_Tab[0] & 0x000000FF);
    cardinfo->SD_cid.ProdName1 = tmp << 24;

    /*!< Byte 4 */
    tmp = (uint8_t)((CID_Tab[1] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ProdName1 |= tmp << 16;

    /*!< Byte 5 */
    tmp = (uint8_t)((CID_Tab[1] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.ProdName1 |= tmp << 8;

    /*!< Byte 6 */
    tmp = (uint8_t)((CID_Tab[1] & 0x0000FF00) >> 8);
    cardinfo->SD_cid.ProdName1 |= tmp;

    /*!< Byte 7 */
    tmp                        = (uint8_t)(CID_Tab[1] & 0x000000FF);
    cardinfo->SD_cid.ProdName2 = tmp;

    /*!< Byte 8 */
    tmp                      = (uint8_t)((CID_Tab[2] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ProdRev = tmp;

    /*!< Byte 9 */
    tmp                     = (uint8_t)((CID_Tab[2] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.ProdSN = tmp << 24;

    /*!< Byte 10 */
    tmp = (uint8_t)((CID_Tab[2] & 0x0000FF00) >> 8);
    cardinfo->SD_cid.ProdSN |= tmp << 16;

    /*!< Byte 11 */
    tmp = (uint8_t)(CID_Tab[2] & 0x000000FF);
    cardinfo->SD_cid.ProdSN |= tmp << 8;

    /*!< Byte 12 */
    tmp = (uint8_t)((CID_Tab[3] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ProdSN |= tmp;

    /*!< Byte 13 */
    tmp = (uint8_t)((CID_Tab[3] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.Reserved1 |= (tmp & 0xF0) >> 4;
    cardinfo->SD_cid.ManufactDate = (tmp & 0x0F) << 8;

    /*!< Byte 14 */
    tmp = (uint8_t)((CID_Tab[3] & 0x0000FF00) >> 8);
    cardinfo->SD_cid.ManufactDate |= tmp;

    /*!< Byte 15 */
    tmp                        = (uint8_t)(CID_Tab[3] & 0x000000FF);
    cardinfo->SD_cid.CID_CRC   = (tmp & 0xFE) >> 1;
    cardinfo->SD_cid.Reserved2 = 1;

    return (errorstatus);
}

/**
 * @brief  Enables wide bus opeartion for the requeseted card if supported by
 *         card.
 * @param WideMode Specifies the SD card wide bus mode.
 *   This parameter can be one of the following values:
 *     @arg SDIO_BUSWIDTH_8B 8-bit data transfer (Only for MMC)
 *     @arg SDIO_BUSWIDTH_4B 4-bit data transfer
 *     @arg SDIO_BUSWIDTH_1B 1-bit data transfer
 * @return SD_Error: SD Card Error code.
 */
SD_Error SD_GetCardStatus(SD_CardStatus* cardstatus)
{
    SD_Error errorstatus = SD_OK;
    uint8_t tmp          = 0;

    errorstatus = SD_SendSDStatus((uint32_t*)SDSTATUS_Tab);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    /*!< Byte 0 */
    tmp                       = (uint8_t)((SDSTATUS_Tab[0] & 0xC0) >> 6);
    cardstatus->DAT_BUS_WIDTH = tmp;

    /*!< Byte 0 */
    tmp                      = (uint8_t)((SDSTATUS_Tab[0] & 0x20) >> 5);
    cardstatus->SECURED_MODE = tmp;

    /*!< Byte 2 */
    tmp                      = (uint8_t)((SDSTATUS_Tab[2] & 0xFF));
    cardstatus->SD_CARD_TYPE = tmp << 8;

    /*!< Byte 3 */
    tmp = (uint8_t)((SDSTATUS_Tab[3] & 0xFF));
    cardstatus->SD_CARD_TYPE |= tmp;

    /*!< Byte 4 */
    tmp                                = (uint8_t)(SDSTATUS_Tab[4] & 0xFF);
    cardstatus->SIZE_OF_PROTECTED_AREA = tmp << 24;

    /*!< Byte 5 */
    tmp = (uint8_t)(SDSTATUS_Tab[5] & 0xFF);
    cardstatus->SIZE_OF_PROTECTED_AREA |= tmp << 16;

    /*!< Byte 6 */
    tmp = (uint8_t)(SDSTATUS_Tab[6] & 0xFF);
    cardstatus->SIZE_OF_PROTECTED_AREA |= tmp << 8;

    /*!< Byte 7 */
    tmp = (uint8_t)(SDSTATUS_Tab[7] & 0xFF);
    cardstatus->SIZE_OF_PROTECTED_AREA |= tmp;

    /*!< Byte 8 */
    tmp                     = (uint8_t)((SDSTATUS_Tab[8] & 0xFF));
    cardstatus->SPEED_CLASS = tmp;

    /*!< Byte 9 */
    tmp                          = (uint8_t)((SDSTATUS_Tab[9] & 0xFF));
    cardstatus->PERFORMANCE_MOVE = tmp;

    /*!< Byte 10 */
    tmp                 = (uint8_t)((SDSTATUS_Tab[10] & 0xF0) >> 4);
    cardstatus->AU_SIZE = tmp;

    /*!< Byte 11 */
    tmp                    = (uint8_t)(SDSTATUS_Tab[11] & 0xFF);
    cardstatus->ERASE_SIZE = tmp << 8;

    /*!< Byte 12 */
    tmp = (uint8_t)(SDSTATUS_Tab[12] & 0xFF);
    cardstatus->ERASE_SIZE |= tmp;

    /*!< Byte 13 */
    tmp                       = (uint8_t)((SDSTATUS_Tab[13] & 0xFC) >> 2);
    cardstatus->ERASE_TIMEOUT = tmp;

    /*!< Byte 13 */
    tmp                      = (uint8_t)((SDSTATUS_Tab[13] & 0x3));
    cardstatus->ERASE_OFFSET = tmp;

    return (errorstatus);
}

/**
 * @brief  Configure the data width of the card (depending on whether the card supports it)
 * @param WideMode Specifies the SD card wide bus mode.
 *   This parameter can be one of the following values:
 *     @arg SDIO_BUSWIDTH_8B 8-bit data transfer (Only for MMC)
 *     @arg SDIO_BUSWIDTH_4B 4-bit data transfer
 *     @arg SDIO_BUSWIDTH_1B 1-bit data transfer
 * @return SD_Error: SD Card Error code.
 */
SD_Error SD_EnableWideBusOperation(uint32_t WideMode)
{
    SD_Error errorstatus = SD_OK;

    /*!< MMC Card */
    if (SDIO_MULTIMEDIA_CARD == CardType)
    {
        errorstatus = MMCSetWideBus(WideMode); // Use acmd6 to set the bus width and the transmission mode of the card

        if (SD_OK == errorstatus)
        {
            /*!< Configure the SDIO peripheral */
#if 0
        SDIO_InitStructure.ClkDiv = SDIO_TRANSFER_CLK_DIV; 
        SDIO_InitStructure.ClkEdge = SDIO_CLKEDGE_RISING;
        SDIO_InitStructure.ClkBypass = SDIO_ClkBYPASS_DISABLE;
        SDIO_InitStructure.ClkPwrSave = SDIO_CLKPOWERSAVE_DISABLE;
        SDIO_InitStructure.BusWidth = WideMode; 
        SDIO_InitStructure.HardwareClkCtrl = SDIO_HARDWARE_CLKCTRL_DISABLE;
#else
            SDIO_InitStructure.BusWidth = WideMode;
#endif

            SDIO_Init(&SDIO_InitStructure);
        }
    }
    else if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType)
             || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
    {
        if (SDIO_BUSWIDTH_8B == WideMode) // 2.0 SD does not support 8bits
        {
            errorstatus = SD_UNSUPPORTED_FEATURE;
            return (errorstatus);
        }
        else if (SDIO_BUSWIDTH_4B == WideMode) // 4 data line mode
        {
            errorstatus = SDEnWideBus(ENABLE); // Use acmd6 to set the bus width and the transmission mode of the card

            if (SD_OK == errorstatus)
            {
                /*!< Configure the SDIO peripheral */
                // SDIO_InitStructure.ClkDiv = SDIO_TRANSFER_CLK_DIV;
                // SDIO_InitStructure.ClkEdge = SDIO_CLKEDGE_RISING;
                // SDIO_InitStructure.ClkBypass = SDIO_ClkBYPASS_DISABLE;
                // SDIO_InitStructure.ClkPwrSave = SDIO_CLKPOWERSAVE_DISABLE;
                SDIO_InitStructure.BusWidth =
                    SDIO_BUSWIDTH_4B; // This is to set the SDIO transmission mode of N32G45X. The switching mode must
                                      // correspond to both the card and SDIO
                // SDIO_InitStructure.HardwareClkCtrl = SDIO_HARDWARE_CLKCTRL_DISABLE;
                SDIO_Init(&SDIO_InitStructure);
            }
        }
        else // Single data line mode
        {
            errorstatus = SDEnWideBus(DISABLE);

            if (SD_OK == errorstatus)
            {
                /*!< Configure the SDIO peripheral */
                // SDIO_InitStructure.ClkDiv = SDIO_TRANSFER_CLK_DIV;
                // SDIO_InitStructure.ClkEdge = SDIO_CLKEDGE_RISING;
                // SDIO_InitStructure.ClkBypass = SDIO_ClkBYPASS_DISABLE;
                // SDIO_InitStructure.ClkPwrSave = SDIO_CLKPOWERSAVE_DISABLE;
                SDIO_InitStructure.BusWidth = SDIO_BUSWIDTH_1B;
                // SDIO_InitStructure.HardwareClkCtrl = SDIO_HARDWARE_CLKCTRL_DISABLE;
                SDIO_Init(&SDIO_InitStructure);
            }
        }
    }

    return (errorstatus);
}

/**
 * @brief  Using cmd7, select the card whose relative address is addr, and deselect other cards
 *         If addr = 0, deselect all cards
 * @param addr address of the select card
 * @return SD_Error: SD Card Error code.
 */
SD_Error SD_SelectDeselect(uint32_t addr)
{
    SD_Error errorstatus = SD_OK;

    /*!< Send CMD7 SDIO_SEL_DESEL_CARD */
    SDIO_CmdInitStructure.CmdArgument  = addr;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SEL_DESEL_CARD;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SEL_DESEL_CARD);
    return (errorstatus);
}

/**
 * @brief  Allows to read one block from a specified address in a card. The Data
 *         transfer can be managed by DMA mode or Polling mode.
 * @note   This operation should be followed by two functions to check if the
 *         DMA Controller and SD Card status.
 *          - SD_ReadWaitOperation(): this function insure that the DMA
 *            controller has finished all data transfer.
 *          - SD_GetStatus(): to check that the SD Card has finished the
 *            data transfer and it is ready for data.
 * @param readbuff pointer to the buffer that will contain the received data
 * @param ReadAddr Address from where data are to be read.
 * @param BlockSize the SD card Data block size. The Block size should be 512.
 * @return SD_Error: SD Card Error code.
 */

u32 g_write_FLAG_DBCKEND;
u32 g_write_FLAG_TXUNDERR;
u32 g_write_FLAG_DCRCFAIL;
u32 g_write_FLAG_DTIMEOUT;
u32 g_write_FLAG_STBITERR;

u32 g_write_FLAG_TXFIFOE;
// u32 g_write_FLAG_TXUNDERR;
u32 g_write_FLAG_TXFIFOF;
u32 g_read_FLAG_RXOVERR;
u32 g_read_FLAG_RXFIFOF;
u32 g_read_FLAG_RXFIFOE;

u32 g_write_FLAG_CCRCFAIL;
u32 g_write_FLAG_CMDREND;
u32 g_write_FLAG_CTIMEOUT;

SD_Error SD_ReadBlock_ForIrqTest(uint8_t* readbuff, uint32_t ReadAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;

    uint32_t count = 0, *tempbuff = (uint32_t*)readbuff;

    g_write_FLAG_DBCKEND  = 0;
    g_write_FLAG_TXUNDERR = 0;
    g_write_FLAG_DCRCFAIL = 0;
    g_write_FLAG_DTIMEOUT = 0;
    g_write_FLAG_STBITERR = 0;
    g_write_FLAG_TXFIFOE  = 0;
    // g_write_FLAG_TXUNDERR = 0;
    g_write_FLAG_TXFIFOF = 0;
    g_read_FLAG_RXOVERR  = 0;
    g_read_FLAG_RXFIFOF  = 0;
    g_read_FLAG_RXFIFOE  = 0;

    g_write_FLAG_CCRCFAIL = 0;
    g_write_FLAG_CMDREND  = 0;
    g_write_FLAG_CTIMEOUT = 0;

    TransferError = SD_OK;
    TransferEnd = 0; // End of transmission flag, set 1 in interrupt service
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        ReadAddr /= 512;
    }
    /********add,Without this section, it is easy to get stuck in DMA detection*****************/
    /*!< Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error_ForIrqTest(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }

    // Enable hardware flow control
    SDIO->CLKCTRL |= 0x4000;

    /*********************************************************************************/
    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = BlockSize;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)9 << 4;
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOSDIO;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< Send CMD17 READ_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)ReadAddr;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_READ_SINGLE_BLOCK;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error_ForIrqTest(SD_CMD_READ_SINGLE_BLOCK);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    /*!< In case of single block transfer, no need of stop transfer at all.*/
    /*!< Polling mode */
    // while (!(SDIO->STS &(SDIO_FLAG_RXORERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_DATBLKEND |
    // SDIO_FLAG_SBERR)))

#if 0
  //while(!(g_write_FLAG_DBCKEND || g_read_FLAG_RXOVERR || g_write_FLAG_DCRCFAIL || g_write_FLAG_DTIMEOUT || g_write_FLAG_STBITERR))     
  while(!(g_write_FLAG_DBCKEND || g_read_FLAG_RXOVERR  || g_write_FLAG_DTIMEOUT || g_write_FLAG_STBITERR))      
  {
    if (SDIO_GetFlag(SDIO_FLAG_RFIFOHF) != RESET)
    {
      //rxfifo full
      //while(!g_read_FLAG_RXFIFOF);

      while(!g_read_FLAG_RXFIFOE)
      {
          *(tempbuff + count) = SDIO_ReadData();
          count++;

          if(count >= 128)
            break;
      }
      if(count >= 128)
        break;

      
      //for (count = 0; count < 8; count++)
      {
        //*(tempbuff + count) = SDIO_ReadData();
      }
      //tempbuff += 8;
    }
  }
#endif
    while (count < 128)
    {
        *(tempbuff + count) = SDIO_ReadData();
        count++;
    }

    // Turn off hardware flow control
    SDIO->CLKCTRL &= ~0x4000;

    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_RXORERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_RXORERR);
        errorstatus = SD_RX_OVERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }

    while (SDIO_GetFlag(SDIO_FLAG_RDATVALID) != RESET)
    // while(SDIO->FIFOCOUNT)
    {
        *tempbuff = SDIO_ReadData();
        tempbuff++;
    }

    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);

    return (errorstatus);
}

SD_Error SD_ReadBlock_ForDblockSize(uint8_t* readbuff, uint32_t ReadAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;
#if defined(SD_POLLING_MODE)
    uint32_t count = 0, *tempbuff = (uint32_t*)readbuff;
#endif
    u32 m;

    TransferError = SD_OK;
    TransferEnd = 0; // End of transmission flag, set 1 in interrupt service
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        ReadAddr /= 512;
    }
    /*******************add,Without this section, it is easy to get stuck in DMA detection**************/
    /*!<Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
    /*********************************************************************************/
    SDIO_DataInitStructure.DatTimeout = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen     = BlockSize;
    for (m = 0; m < 31; m++)
    {
        if (1 << m == BlockSize)
        {
            SDIO_DataInitStructure.DatBlkSize = (uint32_t)m
                                                << 4; // This parameter can be used instead of SDIO datablocksize
            break;
        }
    }

    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOSDIO;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< Send CMD17 READ_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)ReadAddr;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_READ_SINGLE_BLOCK;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_READ_SINGLE_BLOCK);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

#if defined(SD_POLLING_MODE)
    /*!< In case of single block transfer, no need of stop transfer at all.*/
    /*!< Polling mode */
    while (!(SDIO->STS
             & (SDIO_FLAG_RXORERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_DATBLKEND | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_RFIFOHF) != RESET)
        {
            for (count = 0; count < 8; count++)
            {
                *(tempbuff + count) = SDIO_ReadData();
            }
            tempbuff += 8;
        }
    }

    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_RXORERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_RXORERR);
        errorstatus = SD_RX_OVERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }
    while (SDIO_GetFlag(SDIO_FLAG_RDATVALID) != RESET)
    {
        *tempbuff = SDIO_ReadData();
        tempbuff++;
    }

    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);

#elif defined(SD_DMA_MODE)
    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);

    if (BlockSize == 1)
    {
        BlockSize = 4;
    }
    SDIO_DMACmd(ENABLE);
    SD_DMA_RxConfig((uint32_t*)readbuff, BlockSize);
#endif

    return (errorstatus);
}

/**
 * @brief  Allows to read one block from a specified address in a card. The Data
 *         transfer can be managed by DMA mode or Polling mode.
 * @note   This operation should be followed by two functions to check if the
 *         DMA Controller and SD Card status.
 *          - SD_ReadWaitOperation(): this function insure that the DMA
 *            controller has finished all data transfer.
 *          - SD_GetStatus(): to check that the SD Card has finished the
 *            data transfer and it is ready for data.
 * @param readbuff pointer to the buffer that will contain the received data
 * @param ReadAddr Address from where data are to be read.
 * @param BlockSize the SD card Data block size. The Block size should be 512.
 * @return SD_Error: SD Card Error code.
 */
SD_Error SD_ReadBlock_ForDtimer(uint8_t* readbuff, uint32_t ReadAddr, uint16_t BlockSize, uint32_t Dtimeout)
{
    SD_Error errorstatus = SD_OK;
#if defined(SD_POLLING_MODE)
    uint32_t count = 0, *tempbuff = (uint32_t*)readbuff;
#endif

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        ReadAddr /= 512;
    }
    /*******************add,Without this section, it is easy to get stuck in DMA detection**************/
    /*!<Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
    /*********************************************************************************/
    // Dtimer
    SDIO_DataInitStructure.DatTimeout        = Dtimeout; // SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = BlockSize;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)9 << 4;
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOSDIO;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< Send CMD17 READ_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)ReadAddr;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_READ_SINGLE_BLOCK;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_READ_SINGLE_BLOCK);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

#if defined(SD_POLLING_MODE)
    /*!< In case of single block transfer, no need of stop transfer at all.*/
    /*!< Polling mode */
    while (!(SDIO->STS
             & (SDIO_FLAG_RXORERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_DATBLKEND | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_RFIFOHF) != RESET)
        {
            for (count = 0; count < 8; count++)
            {
                *(tempbuff + count) = SDIO_ReadData();
            }
            tempbuff += 8;
        }
    }

    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_RXORERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_RXORERR);
        errorstatus = SD_RX_OVERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }
    while (SDIO_GetFlag(SDIO_FLAG_RDATVALID) != RESET)
    {
        *tempbuff = SDIO_ReadData();
        tempbuff++;
    }

    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);

#elif defined(SD_DMA_MODE)
    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);
    SDIO_DMACmd(ENABLE);
    SD_DMA_RxConfig((uint32_t*)readbuff, BlockSize);
#endif

    return (errorstatus);
}

SD_Error SD_ReadBlock_ForWaitPend(uint8_t* readbuff, uint32_t ReadAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;

    uint32_t count = 0, *tempbuff = (uint32_t*)readbuff;
    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        ReadAddr /= 512;
    }
    /*******************add,Without this section, it is easy to get stuck in DMA detection**************/
    /*!<Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
    /*********************************************************************************/
    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = BlockSize;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)9 << 4;
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOSDIO;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< Send CMD17 READ_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)ReadAddr;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_READ_SINGLE_BLOCK;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_READ_SINGLE_BLOCK);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    /*!< In case of single block transfer, no need of stop transfer at all.*/
    /*!< Polling mode */
    while (!(SDIO->STS
             & (SDIO_FLAG_RXORERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_DATBLKEND | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_RFIFOHF) != RESET)
        {
            for (count = 0; count < 8; count++)
            {
                *(tempbuff + count) = SDIO_ReadData();
            }
            tempbuff += 8;

#if 0
      if(!already_sent)
      {
         already_sent = 1;
          //CMD15 ?waitpend     
          SDIO->CMDARG = (uint32_t) RCA << 16;
          SDIO->CMDCTRL = 0x64d;//0x60f;    

          //CMD15,?WaitPend
          //SDIO->CMDARG = (uint32_t) RCA << 16;
          //SDIO->CMDCTRL = 0x44d;//0x40f;        
      }
#endif
        }
    }

    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_RXORERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_RXORERR);
        errorstatus = SD_RX_OVERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }

    while (SDIO_GetFlag(SDIO_FLAG_RDATVALID) != RESET)
    {
        *tempbuff = SDIO_ReadData();
        tempbuff++;
    }
    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);
    return (errorstatus);
}

SD_Error SD_ReadBlock_ForMMCGetExtCSD(uint8_t* readbuff, uint32_t ReadAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;
#if defined(SD_POLLING_MODE)
    uint32_t count = 0, *tempbuff = (uint32_t*)readbuff;
#endif

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        ReadAddr /= 512;
    }
    /*******************add,Without this section, it is easy to get stuck in DMA detection**************/
    /*!<Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
    /*********************************************************************************/
    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = BlockSize;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)9 << 4;
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOSDIO;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< Send CMD17 READ_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)0;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_HS_SEND_EXT_CSD;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_HS_SEND_EXT_CSD);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

#if defined(SD_POLLING_MODE)
    /*!< In case of single block transfer, no need of stop transfer at all.*/
    /*!< Polling mode */
    while (!(SDIO->STS
             & (SDIO_FLAG_RXORERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_DATBLKEND | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_RFIFOHF) != RESET)
        {
            for (count = 0; count < 8; count++)
            {
                *(tempbuff + count) = SDIO_ReadData();
            }
            tempbuff += 8;
        }
    }

    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_RXORERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_RXORERR);
        errorstatus = SD_RX_OVERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }

    while (SDIO_GetFlag(SDIO_FLAG_RDATVALID) != RESET)
    {
        *tempbuff = SDIO_ReadData();
        tempbuff++;
    }

    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);

#elif defined(SD_DMA_MODE)
    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);
    SDIO_DMACmd(ENABLE);
    SD_DMA_RxConfig((uint32_t*)readbuff, BlockSize);
#endif

    return (errorstatus);
}

SD_Error SD_ReadBlock_ForSCR(uint8_t* readbuff, uint32_t ReadAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;
#if defined(SD_POLLING_MODE)
    uint32_t count = 0, *tempbuff = (uint32_t*)readbuff;
#endif

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        ReadAddr /= 512;
    }
    /*******************add,Without this section, it is easy to get stuck in DMA detection**************/
    /*!<Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)8;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }

    /*********************************************************************************/
    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = 8;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)3 << 4;
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOSDIO;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)(RCA << 16);
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_APP_CMD; // cmd55
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    /*!< Send CMD17 READ_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)0;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SD_APP_SEND_SCR;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SD_APP_SEND_SCR);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

#if defined(SD_POLLING_MODE)
    /*!< In case of single block transfer, no need of stop transfer at all.*/
    /*!< Polling mode */
    while (!(SDIO->STS
             & (SDIO_FLAG_RXORERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_DATBLKEND | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_RFIFOHF) != RESET)
        {
            for (count = 0; count < 8; count++)
            {
                *(tempbuff + count) = SDIO_ReadData();
            }
            tempbuff += 8;
        }
    }

    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_RXORERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_RXORERR);
        errorstatus = SD_RX_OVERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }

    while (SDIO_GetFlag(SDIO_FLAG_RDATVALID) != RESET)
    {
        *tempbuff = SDIO_ReadData();
        tempbuff++;
    }

    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);

#elif defined(SD_DMA_MODE)
    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);
    SDIO_DMACmd(ENABLE);
    SD_DMA_RxConfig((uint32_t*)readbuff, BlockSize);
#endif

    return (errorstatus);
}

/**
 * @brief  Allows to read one block from a specified address in a card. The Data
 *         transfer can be managed by DMA mode or Polling mode.
 * @note   This operation should be followed by two functions to check if the
 *         DMA Controller and SD Card status.
 *          - SD_ReadWaitOperation(): this function insure that the DMA
 *            controller has finished all data transfer.
 *          - SD_GetStatus(): to check that the SD Card has finished the
 *            data transfer and it is ready for data.
 * @param readbuff pointer to the buffer that will contain the received data
 * @param ReadAddr Address from where data are to be read.
 * @param BlockSize the SD card Data block size. The Block size should be 512.
 * @return SD_Error: SD Card Error code.
 */
SD_Error SD_ReadBlock(uint8_t* readbuff, uint32_t ReadAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;
#if defined(SD_POLLING_MODE)
    uint32_t count = 0, *tempbuff = (uint32_t*)readbuff;
#endif

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        ReadAddr /= 512;
    }
    /*******************add,Without this section, it is easy to get stuck in DMA detection**************/
    /*!<Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
    /*********************************************************************************/
    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = BlockSize;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)9 << 4;
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOSDIO;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< Send CMD17 READ_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)ReadAddr;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_READ_SINGLE_BLOCK;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_READ_SINGLE_BLOCK);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

#if defined(SD_POLLING_MODE)
    /*!< In case of single block transfer, no need of stop transfer at all.*/
    /*!< Polling mode */
    while (!(SDIO->STS
             & (SDIO_FLAG_RXORERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_DATBLKEND | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_RFIFOHF) != RESET)
        {
            for (count = 0; count < 8; count++)
            {
                *(tempbuff + count) = SDIO_ReadData();
            }
            tempbuff += 8;
        }
    }

    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_RXORERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_RXORERR);
        errorstatus = SD_RX_OVERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }

    while (SDIO_GetFlag(SDIO_FLAG_RDATVALID) != RESET)
    {
        *tempbuff = SDIO_ReadData();
        tempbuff++;
    }

    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);

#elif defined(SD_DMA_MODE)
    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);
    SDIO_DMACmd(ENABLE);
    SD_DMA_RxConfig((uint32_t*)readbuff, BlockSize);
#endif

    return (errorstatus);
}

SD_Error SD_ReadBlock_ForDMATest(uint8_t* readbuff, uint32_t ReadAddr, uint16_t BlockSize, u32 DMA_ch)
{
    SD_Error errorstatus = SD_OK;
#if defined(SD_POLLING_MODE)
    uint32_t count = 0, *tempbuff = (uint32_t*)readbuff;
#endif

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        ReadAddr /= 512;
    }
    /*******************add,Without this section, it is easy to get stuck in DMA detection**************/
    /*!<Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
    /*********************************************************************************/
    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = BlockSize;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)9 << 4;
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOSDIO;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< Send CMD17 READ_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)ReadAddr;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_READ_SINGLE_BLOCK;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_READ_SINGLE_BLOCK);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

#if defined(SD_POLLING_MODE)
    /*!< In case of single block transfer, no need of stop transfer at all.*/
    /*!< Polling mode */
    while (!(SDIO->STS
             & (SDIO_FLAG_RXORERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_DATBLKEND | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_RFIFOHF) != RESET)
        {
            for (count = 0; count < 8; count++)
            {
                *(tempbuff + count) = SDIO_ReadData();
            }
            tempbuff += 8;
        }
    }

    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_RXORERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_RXORERR);
        errorstatus = SD_RX_OVERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }

    while (SDIO_GetFlag(SDIO_FLAG_RDATVALID) != RESET)
    {
        *tempbuff = SDIO_ReadData();
        tempbuff++;
    }

    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);

#elif defined(SD_DMA_MODE)
    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);
    SDIO_DMACmd(ENABLE);
    SD_DMA_RxConfig_ForDMATest((uint32_t*)readbuff, BlockSize, DMA_ch);
#endif

    return (errorstatus);
}

SD_Error SD_ReadBlock_ForCMD6Mode1(uint32_t* readbuff, uint32_t ReadAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;
#if defined(SD_POLLING_MODE)
    uint32_t count = 0, *tempbuff = (uint32_t*)readbuff;
#endif

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        // BlockSize = 512;
        // ReadAddr /= 512;
    }

    /*******************add,Without this section, it is easy to get stuck in DMA detection**************/
    /*!<Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)64;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }

    /*********************************************************************************/
    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = 64;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)6 << 4;
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOSDIO;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< Send CMD17 READ_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = 0x80ffff01;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_HS_SWITCH;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_HS_SWITCH);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

#if defined(SD_POLLING_MODE)
    /*!< In case of single block transfer, no need of stop transfer at all.*/
    /*!< Polling mode */
    while (!(SDIO->STS
             & (SDIO_FLAG_RXORERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_DATBLKEND | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_RFIFOHF) != RESET)
        {
            for (count = 0; count < 8; count++)
            {
                *(tempbuff + count) = SDIO_ReadData();
            }
            tempbuff += 8;
        }
    }

    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_RXORERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_RXORERR);
        errorstatus = SD_RX_OVERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }

    while (SDIO_GetFlag(SDIO_FLAG_RDATVALID) != RESET)
    {
        *tempbuff = SDIO_ReadData();
        tempbuff++;
    }

    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);

#elif defined(SD_DMA_MODE)
    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);
    SDIO_DMACmd(ENABLE);
    SD_DMA_RxConfig((uint32_t*)readbuff, BlockSize);
#endif

    return (errorstatus);
}

SD_Error SD_ReadBlock_ForCMD6Mode0(uint32_t* readbuff, uint32_t ReadAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;
#if defined(SD_POLLING_MODE)
    uint32_t count = 0, *tempbuff = (uint32_t*)readbuff;
#endif

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        // BlockSize = 512;
        // ReadAddr /= 512;
    }

    /*******************add,Without this section, it is easy to get stuck in DMA detection**************/
    /*!<Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)64;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }

    /*********************************************************************************/
    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = 64;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)6 << 4;
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOSDIO;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< Send CMD17 READ_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = 0x00ffff01;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_HS_SWITCH;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_HS_SWITCH);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

#if defined(SD_POLLING_MODE)
    /*!< In case of single block transfer, no need of stop transfer at all.*/
    /*!< Polling mode */
    while (!(SDIO->STS
             & (SDIO_FLAG_RXORERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_DATBLKEND | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_RFIFOHF) != RESET)
        {
            for (count = 0; count < 8; count++)
            {
                *(tempbuff + count) = SDIO_ReadData();
            }
            tempbuff += 8;
        }
    }

    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_RXORERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_RXORERR);
        errorstatus = SD_RX_OVERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }

    while (SDIO_GetFlag(SDIO_FLAG_RDATVALID) != RESET)
    {
        *tempbuff = SDIO_ReadData();
        tempbuff++;
    }

    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);

#elif defined(SD_DMA_MODE)
    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);
    SDIO_DMACmd(ENABLE);
    SD_DMA_RxConfig((uint32_t*)readbuff, BlockSize);
#endif

    return (errorstatus);
}

SD_Error SD_ReadBlock_ForStreamRead(uint8_t* readbuff, uint32_t ReadAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;
#if defined(SD_POLLING_MODE)
    uint32_t count = 0, *tempbuff = (uint32_t*)readbuff;
#endif

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        ReadAddr /= 512;
    }

#if 1
    /*******************add,Without this section, it is easy to get stuck in DMA detection**************/
    /*!<Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)512;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
#endif

    /*********************************************************************************/
    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = 0; // BlockSize;
    SDIO_DataInitStructure.DatBlkSize        = 0; //(uint32_t) 9 << 4;
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOSDIO;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_STREAM;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< Send CMD17 READ_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)ReadAddr;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_READ_DAT_UNTIL_STOP;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_READ_DAT_UNTIL_STOP);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

#if defined(SD_POLLING_MODE)
    /*!< In case of single block transfer, no need of stop transfer at all.*/
    /*!< Polling mode */
    while (!(SDIO->STS
             & (SDIO_FLAG_RXORERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_DATBLKEND | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_RFIFOHF) != RESET)
        {
            for (count = 0; count < 8; count++)
            {
                *(tempbuff + count) = SDIO_ReadData();
            }
            tempbuff += 8;
        }
    }

    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_RXORERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_RXORERR);
        errorstatus = SD_RX_OVERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }

    while (SDIO_GetFlag(SDIO_FLAG_RDATVALID) != RESET)
    {
        *tempbuff = SDIO_ReadData();
        tempbuff++;
    }

    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);

#elif defined(SD_DMA_MODE)
    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);
    SDIO_DMACmd(ENABLE);
    SD_DMA_RxConfig((uint32_t*)readbuff, BlockSize);
#endif

    return (errorstatus);
}

SD_Error SD_ReadBlock_ForCMD56(uint8_t* readbuff, uint32_t ReadAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;
#if defined(SD_POLLING_MODE)
    uint32_t count = 0, *tempbuff = (uint32_t*)readbuff;
#endif

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;
    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        ReadAddr /= 512;
    }
    /*******************add,Without this section, it is easy to get stuck in DMA detection**************/
    /*!<Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);
    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
    /*********************************************************************************/
    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = BlockSize;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)9 << 4;
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOSDIO;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< Send CMD17 READ_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = 1; //(uint32_t)ReadAddr;//host get block from card
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_GEN_CMD;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);
    errorstatus = CmdResp1Error(SD_CMD_GEN_CMD);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

#if defined(SD_POLLING_MODE)
    /*!< In case of single block transfer, no need of stop transfer at all.*/
    /*!< Polling mode */
    while (!(SDIO->STS
             & (SDIO_FLAG_RXORERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_DATBLKEND | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_RFIFOHF) != RESET)
        {
            for (count = 0; count < 8; count++)
            {
                *(tempbuff + count) = SDIO_ReadData();
            }
            tempbuff += 8;
        }
    }

    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_RXORERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_RXORERR);
        errorstatus = SD_RX_OVERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }

    while (SDIO_GetFlag(SDIO_FLAG_RDATVALID) != RESET)
    {
        *tempbuff = SDIO_ReadData();
        tempbuff++;
    }

    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);

#elif defined(SD_DMA_MODE)
    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);
    SDIO_DMACmd(ENABLE);
    SD_DMA_RxConfig((uint32_t*)readbuff, BlockSize);
#endif

    return (errorstatus);
}

/**
 * @brief  Allows to read blocks from a specified address  in a card.  The Data
 *         transfer can be managed by DMA mode or Polling mode. //?????
 * @note   This operation should be followed by two functions to check if the
 *         DMA Controller and SD Card status.      //dma????????????
 *          - SD_ReadWaitOperation(): this function insure that the DMA
 *            controller has finished all data transfer.
 *          - SD_GetStatus(): to check that the SD Card has finished the
 *            data transfer and it is ready for data.
 * @param readbuff pointer to the buffer that will contain the received data.
 * @param ReadAddr Address from where data are to be read.
 * @param BlockSize the SD card Data block size. The Block size should be 512.
 * @param NumberOfBlocks number of blocks to be read.
 * @return SD_Error: SD Card Error code.
 */
SD_Error SD_ReadMultiBlocks(uint8_t* readbuff, uint32_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
    SD_Error errorstatus = SD_OK;
    TransferError        = SD_OK;
    TransferEnd          = 0;
    StopCondition        = 1;

    SDIO->DATCTRL = 0x0; // Reset data control register

    if (CardType
        == SDIO_HIGH_CAPACITY_SD_CARD) // The address of the SDHC card that is in blocks with 512 bytes each block
    {
        BlockSize = 512;
        ReadAddr /= 512;
    }

    /*!< Set Block Size for Card,cmd16,
          If it is an SDSC card, it can be used to set the block size.
          If it is an SDHC card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }

    SDIO_DataInitStructure.DatTimeout = SD_DATATIMEOUT; // Wait timeout limit
    SDIO_DataInitStructure.DatLen =
        NumberOfBlocks * BlockSize; // For block data transfer, the value in the data length register must be a multiple
                                    // of the data block length (see SDIO_datctrl)
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)9 << 4;
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOSDIO; // Transmission direction
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK; // Transmission mode
    SDIO_DataInitStructure.DPSMConfig = SDIO_DPSM_ENABLE;            // enable the data state machine
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< Send CMD18 READ_MULT_BLOCK with argument data address */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)ReadAddr;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_READ_MULT_BLOCK;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_READ_MULT_BLOCK);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE); // Start data transmission end interrupt
    SDIO_DMACmd(ENABLE);
    SD_DMA_RxConfig((uint32_t*)readbuff, (NumberOfBlocks * BlockSize)); // config DMA

    return (errorstatus);
}

/**
 * @brief  This function waits until the SDIO DMA data transfer is finished.
 *         This function should be called after SDIO_ReadMultiBlocks() function
 *         to insure that all data sent by the card are already transferred by
 *         the DMA controller.
 * @return SD_Error: SD Card Error code.
 */
SD_Error SD_WaitReadOperation(void)
{
    SD_Error errorstatus = SD_OK;
#if defined(SD_POLLING_MODE)
    while (!(SDIO->STS & SDIO_INT_DATEND))
    {
    }

#elif defined(SD_DMA_MODE)
    // Wait for DMA transmission to finish
    while ((SD_DMAEndOfTransferStatus() == RESET) && (TransferEnd == 0) && (TransferError == SD_OK))
    {
    }

    if (TransferError != SD_OK)
    {
        return (TransferError);
    }
#endif
    return (errorstatus);
}

/**
 * @brief  Allows to write one block starting from a specified address in a card.
 *         The Data transfer can be managed by DMA mode or Polling mode.
 * @note   This operation should be followed by two functions to check if the
 *         DMA Controller and SD Card status.
 *          - SD_ReadWaitOperation(): this function insure that the DMA
 *            controller has finished all data transfer.
 *          - SD_GetStatus(): to check that the SD Card has finished the
 *            data transfer and it is ready for data.
 * @param writebuff pointer to the buffer that contain the data to be transferred.
 * @param WriteAddr Address from where data are to be read.
 * @param BlockSize the SD card Data block size. The Block size should be 512.
 * @return SD_Error: SD Card Error code.
 */

SD_Error SD_WriteBlock_ForIrqTest(uint8_t* writebuff, uint32_t WriteAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;

    uint32_t bytestransferred = 0, count = 0, restwords = 0;
    uint32_t* tempbuff = (uint32_t*)writebuff;

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    g_write_FLAG_DBCKEND  = 0;
    g_write_FLAG_TXUNDERR = 0;
    g_write_FLAG_DCRCFAIL = 0;
    g_write_FLAG_DTIMEOUT = 0;
    g_write_FLAG_STBITERR = 0;
    g_write_FLAG_TXFIFOE  = 0;
    // g_write_FLAG_TXUNDERR = 0;
    g_write_FLAG_TXFIFOF = 0;

    g_write_FLAG_CCRCFAIL = 0;
    g_write_FLAG_CMDREND  = 0;
    g_write_FLAG_CTIMEOUT = 0;

    SDIO->DATCTRL = 0x0;
    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        WriteAddr /= 512;
    }

    /******Without this section, it is easy to get stuck in DMA detection************/
    /*!< Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error_ForIrqTest(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
    /*********************************************************************************/

    /*!< Send CMD24 WRITE_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = WriteAddr; //????
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_WRITE_SINGLE_BLOCK;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error_ForIrqTest(SD_CMD_WRITE_SINGLE_BLOCK);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    // Enable hardware flow control
    SDIO->CLKCTRL |= 0x4000;

    // Configure SDIO write data register
    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = BlockSize;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)9 << 4; // SDIO_DATBLK_SIZE_512B
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOCARD;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig = SDIO_DPSM_ENABLE; // Open data channel state machine
    SDIO_ConfigData(&SDIO_DataInitStructure);

#if 1
    /*!< In case of single data block transfer no need of stop command at all */
    // while(!(g_write_FLAG_DBCKEND || g_write_FLAG_TXUNDERR || g_write_FLAG_DCRCFAIL || g_write_FLAG_DTIMEOUT ||
    // g_write_FLAG_STBITERR))

    // Because hardware flow control is turned on, data CRC error will be reported, and data CRC error detection will be
    // removed while(!(g_write_FLAG_DBCKEND || g_write_FLAG_TXUNDERR  || g_write_FLAG_DTIMEOUT || g_write_FLAG_STBITERR))
    while (!(SDIO->STS & (SDIO_FLAG_DATBLKEND | SDIO_FLAG_TXURERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_SBERR)))
    {
        // if (SDIO_GetFlag(SDIO_FLAG_TFIFOHE) != RESET)
        {
            if ((512 - bytestransferred) < 32)
            {
                restwords = ((512 - bytestransferred) % 4 == 0) ? ((512 - bytestransferred) / 4)
                                                                : ((512 - bytestransferred) / 4 + 1);
                for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
                {
                    SDIO_WriteData(*tempbuff);
                }
                break;
            }
            else
            {
// Construct tx fifo full interrupt, once satisfied, it will no longer enter
#if 0
        count = 0;
        while(!g_write_FLAG_TXFIFOF)
        {
            SDIO_WriteData(*(tempbuff + count));
            count++;
            tempbuff++;
            bytestransferred += 4;
        }
#endif

                for (count = 0; count < 32; count++)
                {
                    SDIO_WriteData(*(tempbuff + count));
                }
                tempbuff += 32;
                bytestransferred += 128;
            }
        }
    }
#endif

    // Turn off hardware flow control
    SDIO->CLKCTRL &= ~0x4000;

    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_TXURERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_TXURERR);
        errorstatus = SD_TX_UNDERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }

    return (errorstatus);
}

SD_Error SD_WriteBlock_ForWaitInt(uint8_t* writebuff, uint32_t WriteAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;

    uint32_t bytestransferred = 0, count = 0, restwords = 0;
    uint32_t* tempbuff = (uint32_t*)writebuff;

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        WriteAddr /= 512;
    }

    /******Without this section, it is easy to get stuck in DMA detection************/
    /*!< Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1

    //
    SDIO_CmdInitStructure.WaitType   = SDIO_WAIT_INT;
    SDIO_CmdInitStructure.CPSMConfig = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
    /*********************************************************************************/

    /*!< Send CMD24 WRITE_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = WriteAddr; //????
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_WRITE_SINGLE_BLOCK;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_WRITE_SINGLE_BLOCK);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }
    // Configure SDIO write data register
    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = BlockSize;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)9 << 4; // SDIO_DATBLK_SIZE_512B
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOCARD;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< In case of single data block transfer no need of stop command at all */
    // PrintLog("1fifocnt = %d", SDIO->FIFOCOUNT);
    // already_sent = SDIO->FIFOCOUNT;
    while (!(SDIO->STS
             & (SDIO_FLAG_DATBLKEND | SDIO_FLAG_TXURERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_TFIFOHE) != RESET)
        {
            if ((512 - bytestransferred) < 32)
            {
                restwords = ((512 - bytestransferred) % 4 == 0) ? ((512 - bytestransferred) / 4)
                                                                : ((512 - bytestransferred) / 4 + 1);
                for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
                {
                    SDIO_WriteData(*tempbuff);
                }
            }
            else
            {
                for (count = 0; count < 16; count++)
                {
                    SDIO_WriteData(*(tempbuff + count));
                }
                tempbuff += 16;
                bytestransferred += 64;
                // Perform a calculation of fifocnt here
            }
        }
    }
    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_TXURERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_TXURERR);
        errorstatus = SD_TX_UNDERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }
    return (errorstatus);
}

SD_Error SD_WriteBlock_ForWaitPend(uint8_t* writebuff, uint32_t WriteAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;

    uint32_t bytestransferred = 0, count = 0, restwords = 0;
    uint32_t* tempbuff = (uint32_t*)writebuff;
    u32 already_sent   = 0;

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        WriteAddr /= 512;
    }

    /******Without this section, it is easy to get stuck in DMA detection************/
    /*!< Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
    /*********************************************************************************/

    /*!< Send CMD24 WRITE_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = WriteAddr;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_WRITE_SINGLE_BLOCK;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_WRITE_SINGLE_BLOCK);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = BlockSize;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)9 << 4; // SDIO_DATBLK_SIZE_512B
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOCARD;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< In case of single data block transfer no need of stop command at all */
    // already_sent = SDIO->FIFOCOUNT;
    while (!(SDIO->STS
             & (SDIO_FLAG_DATBLKEND | SDIO_FLAG_TXURERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_TFIFOHE) != RESET)
        {
            if ((512 - bytestransferred) < 32)
            {
                restwords = ((512 - bytestransferred) % 4 == 0) ? ((512 - bytestransferred) / 4)
                                                                : ((512 - bytestransferred) / 4 + 1);
                for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
                {
                    SDIO_WriteData(*tempbuff);
                }
            }
            else
            {
                for (count = 0; count < 16; count++)
                {
                    SDIO_WriteData(*(tempbuff + count));
                }
                tempbuff += 16;
                bytestransferred += 64;
                // Perform a calculation of fifocnt here

#if 1
                if (!already_sent)
                {
                    already_sent = 1;

                    // CMD15 open waitpend
                    SDIO->CMDARG  = (uint32_t)RCA << 16;
                    SDIO->CMDCTRL = 0x64d; // 0x60f;

                    // CMD15,close WaitPend
                    // SDIO->CMDARG = (uint32_t) RCA << 16;
                    // SDIO->CMDCTRL = 0x44d;//0x40f;
                }
#endif
            }
        }
    }
    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_TXURERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_TXURERR);
        errorstatus = SD_TX_UNDERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }
    return (errorstatus);
}

SD_Error SD_WriteBlock_ForFIFOCnt(uint8_t* writebuff, uint32_t WriteAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;

    uint32_t bytestransferred = 0, count = 0, restwords = 0;
    uint32_t* tempbuff = (uint32_t*)writebuff;

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        WriteAddr /= 512;
    }

    /******Without this section, it is easy to get stuck in DMA detection************/
    /*!< Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
    /*********************************************************************************/

    /*!< Send CMD24 WRITE_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = WriteAddr;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_WRITE_SINGLE_BLOCK;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_WRITE_SINGLE_BLOCK);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    // Enable hardware flow control
    // SDIO->CLKCTRL |= 0x4000;

    // Configure SDIO write data register
    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = BlockSize;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)9 << 4; // SDIO_DATBLK_SIZE_512B
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOCARD;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< In case of single data block transfer no need of stop command at all */
    while (!SDIO->FIFOCOUNT)
        ;
    // already_sent = SDIO->FIFOCOUNT;
    while (!(SDIO->STS
             & (SDIO_FLAG_DATBLKEND | SDIO_FLAG_TXURERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_TFIFOHE) != RESET)
        {
            if ((512 - bytestransferred) < 32)
            {
                restwords = ((512 - bytestransferred) % 4 == 0) ? ((512 - bytestransferred) / 4)
                                                                : ((512 - bytestransferred) / 4 + 1);
                for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
                {
                    SDIO_WriteData(*tempbuff);
                }
            }
            else
            {
                for (count = 0; count < 8; count++)
                {
                    SDIO_WriteData(*(tempbuff + count));
                }
                tempbuff += 8;
                bytestransferred += 32;
                // Perform a calculation of fifocnt here

#if 0
                if (!already_sent)
                {
                    already_sent = 1;
                    // for (count = 0; count < 255; count++)
                    while ((8 + SDIO->FIFOCOUNT) != 128)
                    {
                    }

                    if ((8 + SDIO->FIFOCOUNT) != 128)
                    {
                        errorstatus = 0x5a5a;
                        return (errorstatus);
                    }
                }
#endif
            }
        }
    }

    // Turn off hardware flow control
    // SDIO->CLKCTRL &= ~0x4000;

    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_TXURERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_TXURERR);
        errorstatus = SD_TX_UNDERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }
    return (errorstatus);
}

SD_Error SD_WriteBlock_ForDblockSize(uint8_t* writebuff, uint32_t WriteAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;

#if defined(SD_POLLING_MODE)
    uint32_t bytestransferred = 0, count = 0, restwords = 0;
    uint32_t* tempbuff = (uint32_t*)writebuff;
#endif
    u32 m;

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        WriteAddr /= 512;
    }

    /******Without this section, it is easy to get stuck in DMA detection************/
    /*!< Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
    /*********************************************************************************/

    /*!< Send CMD24 WRITE_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = WriteAddr;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_WRITE_SINGLE_BLOCK;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_WRITE_SINGLE_BLOCK);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    SDIO_DataInitStructure.DatTimeout = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen     = BlockSize;

    for (m = 0; m < 31; m++)
    {
        if (1 << m == BlockSize)
        {
            SDIO_DataInitStructure.DatBlkSize = (uint32_t)m << 4; // SDIO_DATBLK_SIZE_512B
            break;
        }
    }

    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOCARD;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< In case of single data block transfer no need of stop command at all */
#if defined(SD_POLLING_MODE)
    while (!(SDIO->STS
             & (SDIO_FLAG_DATBLKEND | SDIO_FLAG_TXURERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_TFIFOHE) != RESET)
        {
            if ((512 - bytestransferred) < 32)
            {
                restwords = ((512 - bytestransferred) % 4 == 0) ? ((512 - bytestransferred) / 4)
                                                                : ((512 - bytestransferred) / 4 + 1);
                for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
                {
                    SDIO_WriteData(*tempbuff);
                }
            }
            else
            {
                for (count = 0; count < 8; count++)
                {
                    SDIO_WriteData(*(tempbuff + count));
                }
                tempbuff += 8;
                bytestransferred += 32;
            }
        }
    }
    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_TXURERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_TXURERR);
        errorstatus = SD_TX_UNDERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }
#elif defined(SD_DMA_MODE)
    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);  //????????

    if (BlockSize == 1)
    {
        BlockSize = 4;
    }
    SD_DMA_TxConfig((uint32_t*)writebuff, BlockSize);
    SDIO_DMACmd(ENABLE);
#endif

    return (errorstatus);
}

SD_Error SD_WriteBlock_ForLockUnlock(uint8_t* writebuff, uint16_t BlockSize)
{
    u32 m;
    SD_Error errorstatus = SD_OK;
    SDIO->DATCTRL        = 0x0;

    /******Without this section, it is easy to get stuck in DMA detection************/
    /*!< Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
    /*********************************************************************************/

    /*!< Send CMD42 */
    SDIO_CmdInitStructure.CmdArgument  = 0x0;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_LOCK_UNLOCK;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_LOCK_UNLOCK);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    SDIO_DataInitStructure.DatTimeout = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen     = BlockSize;

    for (m = 0; m < 31; m++)
    {
        if (1 << m == BlockSize)
        {
            SDIO_DataInitStructure.DatBlkSize = (uint32_t)m << 4;
            break;
        }
    }

    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOCARD;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< In case of single data block transfer no need of stop command at all */
    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);

    if (BlockSize == 1)
    {
        BlockSize = 4;
    }

    SD_DMA_TxConfig((uint32_t*)writebuff, BlockSize);

    SDIO_DMACmd(ENABLE);
    return (errorstatus);
}

SD_Error SD_WriteBlock_ForCMD56(uint8_t* writebuff, uint32_t WriteAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;

#if defined(SD_POLLING_MODE)
    uint32_t bytestransferred = 0, count = 0, restwords = 0;
    uint32_t* tempbuff = (uint32_t*)writebuff;
#endif
    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        WriteAddr /= 512;
    }

    /******Without this section, it is easy to get stuck in DMA detection************/
    /*!< Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
    /*********************************************************************************/
    /*!< Send CMD24 WRITE_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = 0; // WriteAddr;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_GEN_CMD;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_GEN_CMD);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = BlockSize;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)9 << 4; // SDIO_DATBLK_SIZE_512B
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOCARD;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< In case of single data block transfer no need of stop command at all */
#if defined(SD_POLLING_MODE)
    while (!(SDIO->STS
             & (SDIO_FLAG_DATBLKEND | SDIO_FLAG_TXURERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_TFIFOHE) != RESET)
        {
            if ((512 - bytestransferred) < 32)
            {
                restwords = ((512 - bytestransferred) % 4 == 0) ? ((512 - bytestransferred) / 4)
                                                                : ((512 - bytestransferred) / 4 + 1);
                for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
                {
                    SDIO_WriteData(*tempbuff);
                }
            }
            else
            {
                for (count = 0; count < 8; count++)
                {
                    SDIO_WriteData(*(tempbuff + count));
                }
                tempbuff += 8;
                bytestransferred += 32;
            }
        }
    }
    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_TXURERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_TXURERR);
        errorstatus = SD_TX_UNDERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }
#elif defined(SD_DMA_MODE)
    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);
    SD_DMA_TxConfig((uint32_t*)writebuff, BlockSize);
    SDIO_DMACmd(ENABLE);
#endif

    return (errorstatus);
}

SD_Error SD_WriteBlock_ForProgrameCSD(uint8_t* writebuff, uint32_t WriteAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;

#if defined(SD_POLLING_MODE)
    uint32_t bytestransferred = 0, count = 0, restwords = 0;
    uint32_t* tempbuff = (uint32_t*)writebuff;
#endif
    u32 m;

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        WriteAddr /= 512;
    }

#if 0
  /******Without this section, it is easy to get stuck in DMA detection************/
  /*!< Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC card, the block size is 512 bytes, which is not affected by cmd16*/
  SDIO_CmdInitStructure.CmdArgument = (uint32_t) BlockSize;
  SDIO_CmdInitStructure.CmdIndex = SD_CMD_SET_BLOCKLEN;
  SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;   //r1
  SDIO_CmdInitStructure.WaitType = SDIO_WAIT_NO;
  SDIO_CmdInitStructure.CPSMConfig = SDIO_CPSM_ENABLE;
  SDIO_SendCmd(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

  if (SD_OK != errorstatus)
  {
    return(errorstatus);
  }
 /*********************************************************************************/
#endif
    /*!< Send CMD24 WRITE_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = 0x0;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_PROG_CSD;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_PROG_CSD);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    SDIO_DataInitStructure.DatTimeout = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen     = BlockSize;

    for (m = 0; m < 31; m++)
    {
        if (1 << m == BlockSize)
        {
            SDIO_DataInitStructure.DatBlkSize = (uint32_t)m << 4;
            break;
        }
    }

    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOCARD;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< In case of single data block transfer no need of stop command at all */
#if defined(SD_POLLING_MODE)
    while (!(SDIO->STS
             & (SDIO_FLAG_DATBLKEND | SDIO_FLAG_TXURERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_TFIFOHE) != RESET)
        {
            if ((BlockSize - bytestransferred) < 32)
            {
                restwords = ((512 - bytestransferred) % 4 == 0) ? ((512 - bytestransferred) / 4)
                                                                : ((512 - bytestransferred) / 4 + 1);
                for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
                {
                    SDIO_WriteData(*tempbuff);
                }
            }
            else
            {
                for (count = 0; count < 8; count++)
                {
                    SDIO_WriteData(*(tempbuff + count));
                }
                tempbuff += 8;
                bytestransferred += 32;
            }
        }
    }
    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_TXURERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_TXURERR);
        errorstatus = SD_TX_UNDERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }
#elif defined(SD_DMA_MODE)
    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);
    if (BlockSize == 1)
    {
        BlockSize = 4;
    }
    SD_DMA_TxConfig((uint32_t*)writebuff, BlockSize);
    SDIO_DMACmd(ENABLE);
#endif

    return (errorstatus);
}

/**
 * @brief  Allows to write one block starting from a specified address in a card.
 *         The Data transfer can be managed by DMA mode or Polling mode.
 * @note   This operation should be followed by two functions to check if the
 *         DMA Controller and SD Card status.
 *          - SD_ReadWaitOperation(): this function insure that the DMA
 *            controller has finished all data transfer.
 *          - SD_GetStatus(): to check that the SD Card has finished the
 *            data transfer and it is ready for data.
 * @param writebuff pointer to the buffer that contain the data to be transferred.
 * @param WriteAddr Address from where data are to be read.
 * @param BlockSize the SD card Data block size. The Block size should be 512.
 * @return SD_Error: SD Card Error code.
 */
SD_Error SD_WriteBlock(uint8_t* writebuff, uint32_t WriteAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;
#if defined(SD_POLLING_MODE)
    uint32_t bytestransferred = 0, count = 0, restwords = 0;
    uint32_t* tempbuff = (uint32_t*)writebuff;
#endif

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;
    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        WriteAddr /= 512;
    }

    /******Without this section, it is easy to get stuck in DMA detection************/
    /*!< Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
    /*********************************************************************************/

    /*!< Send CMD24 WRITE_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = WriteAddr;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_WRITE_SINGLE_BLOCK;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_WRITE_SINGLE_BLOCK);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = BlockSize;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)9 << 4;
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOCARD;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< In case of single data block transfer no need of stop command at all */
#if defined(SD_POLLING_MODE) 
    while (!(SDIO->STS
             & (SDIO_FLAG_DATBLKEND | SDIO_FLAG_TXURERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_TFIFOHE) != RESET)
        {
            if ((512 - bytestransferred) < 32)
            {
                restwords = ((512 - bytestransferred) % 4 == 0) ? ((512 - bytestransferred) / 4)
                                                                : ((512 - bytestransferred) / 4 + 1);
                for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
                {
                    SDIO_WriteData(*tempbuff);
                }
            }
            else
            {
                for (count = 0; count < 8; count++)
                {
                    SDIO_WriteData(*(tempbuff + count));
                }
                tempbuff += 8;
                bytestransferred += 32;
            }
        }
    }
    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_TXURERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_TXURERR);
        errorstatus = SD_TX_UNDERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }
#elif defined(SD_DMA_MODE)
    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);
    SDIO_DMACmd(ENABLE);
    SD_DMA_TxConfig((uint32_t*)writebuff, BlockSize);
#endif

    return (errorstatus);
}

SD_Error SD_WriteBlock_ForDMATest(uint8_t* writebuff, uint32_t WriteAddr, uint16_t BlockSize, u32 DMA_ch)
{
    SD_Error errorstatus = SD_OK;
#if defined(SD_POLLING_MODE)
    uint32_t bytestransferred = 0, count = 0, restwords = 0;
    uint32_t* tempbuff = (uint32_t*)writebuff;
#endif

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        WriteAddr /= 512;
    }

    /******Without this section, it is easy to get stuck in DMA detection************/
    /*!< Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
    /*********************************************************************************/
    /*!< Send CMD24 WRITE_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = WriteAddr;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_WRITE_SINGLE_BLOCK;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_WRITE_SINGLE_BLOCK);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = BlockSize;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)9 << 4;
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOCARD;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< In case of single data block transfer no need of stop command at all */
#if defined(SD_POLLING_MODE)
    while (!(SDIO->STS
             & (SDIO_FLAG_DATBLKEND | SDIO_FLAG_TXURERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_TFIFOHE) != RESET)
        {
            if ((512 - bytestransferred) < 32)
            {
                restwords = ((512 - bytestransferred) % 4 == 0) ? ((512 - bytestransferred) / 4)
                                                                : ((512 - bytestransferred) / 4 + 1);
                for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
                {
                    SDIO_WriteData(*tempbuff);
                }
            }
            else
            {
                for (count = 0; count < 8; count++)
                {
                    SDIO_WriteData(*(tempbuff + count));
                }
                tempbuff += 8;
                bytestransferred += 32;
            }
        }
    }
    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_TXURERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_TXURERR);
        errorstatus = SD_TX_UNDERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }
#elif defined(SD_DMA_MODE)
    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);
    SD_DMA_TxConfig_ForDMATest((uint32_t*)writebuff, BlockSize, DMA_ch);
    SDIO_DMACmd(ENABLE);
#endif

    return (errorstatus);
}

SD_Error SD_WriteBlock_ForStreamWrite(uint8_t* writebuff, uint32_t WriteAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;
#if defined(SD_POLLING_MODE)
    uint32_t bytestransferred = 0, count = 0, restwords = 0;
    uint32_t* tempbuff = (uint32_t*)writebuff;
#endif

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        WriteAddr /= 512;
    }

#if 1
    /******Without this section, it is easy to get stuck in DMA detection************/
    /*!< Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)512;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
#endif

    /*!< Send CMD24 WRITE_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = WriteAddr;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_WRITE_DAT_UNTIL_STOP;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_WRITE_DAT_UNTIL_STOP);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = BlockSize;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)9 << 4; // SDIO_DATBLK_SIZE_512B
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOCARD;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_STREAM;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< In case of single data block transfer no need of stop command at all */
#if defined(SD_POLLING_MODE)
    while (!(SDIO->STS
             & (SDIO_FLAG_DATBLKEND | SDIO_FLAG_TXURERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_TFIFOHE) != RESET)
        {
            if ((512 - bytestransferred) < 32)
            {
                restwords = ((512 - bytestransferred) % 4 == 0) ? ((512 - bytestransferred) / 4)
                                                                : ((512 - bytestransferred) / 4 + 1);
                for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
                {
                    SDIO_WriteData(*tempbuff);
                }
            }
            else
            {
                for (count = 0; count < 8; count++)
                {
                    SDIO_WriteData(*(tempbuff + count));
                }
                tempbuff += 8;
                bytestransferred += 32;
            }
        }
    }
    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_TXURERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_TXURERR);
        errorstatus = SD_TX_UNDERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }
#elif defined(SD_DMA_MODE)
    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);
    SD_DMA_TxConfig((uint32_t*)writebuff, BlockSize);
    SDIO_DMACmd(ENABLE);
#endif

    return (errorstatus);
}

SD_Error SD_CMD12(void)
{
    SD_Error errorstatus = SD_OK;

    /*!< Send CMD7 SDIO_SEL_DESEL_CARD */
    SDIO_CmdInitStructure.CmdArgument  = 0x0;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_STOP_TRANSMISSION;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SEL_DESEL_CARD);
    return (errorstatus);
}

SD_Error SD_CMD16_BlockSize(u32 block_size)
{
    SD_Error errorstatus               = SD_OK;
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)block_size;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }
    return SD_OK;
}

SD_Error SD_SendACMDS(uint8_t* writebuff, uint32_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
    (void)BlockSize;
    SD_Error errorstatus = SD_OK;
    __IO uint32_t count  = 0;

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 1;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        WriteAddr /= 512;
    }

    /*********************************************************************************/

    /*!< To improve performance  */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)(RCA << 16);
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_APP_CMD; // cmd55
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    /*!< To improve performance */ //  pre-erased,This command can be sent to pre erase when writing multiple blocks
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)0;
    SDIO_CmdInitStructure.CmdIndex     = 22; // cmd22
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCK_COUNT);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }
    return (errorstatus);
}

/**
 * @brief  Allows to write blocks starting from a specified address in a card.
 *         The Data transfer can be managed by DMA mode only.
 * @note   This operation should be followed by two functions to check if the
 *         DMA Controller and SD Card status.
 *          - SD_ReadWaitOperation(): this function insure that the DMA
 *            controller has finished all data transfer.
 *          - SD_GetStatus(): to check that the SD Card has finished the
 *            data transfer and it is ready for data.
 * @param WriteAddr Address from where data are to be read.
 * @param writebuff pointer to the buffer that contain the data to be transferred.
 * @param BlockSize the SD card Data block size. The Block size should be 512.
 * @param NumberOfBlocks number of blocks to be written.
 * @return SD_Error: SD Card Error code.
 */

SD_Error SD_WriteMultiBlocks(uint8_t* writebuff, uint32_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
    SD_Error errorstatus = SD_OK;
    __IO uint32_t count  = 0;

    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 1;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        WriteAddr /= 512;
    }

    /******Without this section, it is easy to get stuck in DMA detection************/
    /*!< Set Block Size for Card,cmd16,If it is an SDSC card, it can be used to set the block size. If it is an SDHC
     * card, the block size is 512 bytes, which is not affected by cmd16*/
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }

    if (SDIO_MULTIMEDIA_CARD != CardType) // Does MMC support acmd23
    {
        /*********************************************************************************/

        /*!< To improve performance  */
        SDIO_CmdInitStructure.CmdArgument  = (uint32_t)(RCA << 16);
        SDIO_CmdInitStructure.CmdIndex     = SD_CMD_APP_CMD; // cmd55
        SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
        SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
        SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
        SDIO_SendCmd(&SDIO_CmdInitStructure);

        errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

        if (errorstatus != SD_OK)
        {
            return (errorstatus);
        }
        /*!< To improve performance */ //  pre-erased,This command can be sent to pre erase when writing multiple blocks
        SDIO_CmdInitStructure.CmdArgument  = (uint32_t)NumberOfBlocks;
        SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCK_COUNT; // cmd23
        SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
        SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
        SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
        SDIO_SendCmd(&SDIO_CmdInitStructure);

        errorstatus = CmdResp1Error(SD_CMD_SET_BLOCK_COUNT);

        if (errorstatus != SD_OK)
        {
            return (errorstatus);
        }
    }

    /*!< Send CMD25 WRITE_MULT_BLOCK with argument data address */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)WriteAddr;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_WRITE_MULT_BLOCK;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_WRITE_MULT_BLOCK);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }

    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = NumberOfBlocks * BlockSize;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)9 << 4;
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOCARD;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);
    SDIO_DMACmd(ENABLE);
    SD_DMA_TxConfig((uint32_t*)writebuff, (NumberOfBlocks * BlockSize));

    return (errorstatus);
}

/**
 * @brief  This function waits until the SDIO DMA data transfer is finished.
 *         This function should be called after SDIO_WriteBlock() and
 *         SDIO_WriteMultiBlocks() function to insure that all data sent by the
 *         card are already transferred by the DMA controller.
 * @return SD_Error: SD Card Error code.
 */
SD_Error SD_WaitWriteOperation(void)
{
    SD_Error errorstatus = SD_OK;

#if defined(SD_DMA_MODE)
    while ((SD_DMAEndOfTransferStatus() == RESET) && (TransferEnd == 0) && (TransferError == SD_OK))
    {
    }
    if (TransferError != SD_OK)
    {
        return (TransferError);
    }
#endif
    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);

    return (errorstatus);
}

/**
 * @brief  Gets the cuurent data transfer state.
 * @return SDTransferState: Data Transfer state.
 *   This value can be:
 *        - SD_TRANSFER_OK: No data transfer is acting
 *        - SD_TRANSFER_BUSY: Data transfer is acting
 */
SDTransferState SD_GetTransferState(void)
{
    if (SDIO->STS & (SDIO_FLAG_TXRUN | SDIO_FLAG_RXRUN))
    {
        return (SD_TRANSFER_BUSY);
    }
    else
    {
        return (SD_TRANSFER_OK);
    }
}

/**
 * @brief  Aborts an ongoing data transfer.
 * @return SD_Error: SD Card Error code.
 */
SD_Error SD_StopTransfer(void)
{
    SD_Error errorstatus = SD_OK;

    /*!< Send CMD12 STOP_TRANSMISSION  */
    SDIO->CMDARG  = 0x0;
    SDIO->CMDCTRL = 0x44C;
    errorstatus   = CmdResp1Error(SD_CMD_STOP_TRANSMISSION);

    return (errorstatus);
}

/**
 * @brief  Allows to erase memory area specified for the given card.
 * @param startaddr the start address.
 * @param endaddr the end address.
 * @return SD_Error: SD Card Error code.
 */
SD_Error SD_Erase(uint32_t startaddr, uint32_t endaddr)
{
    SD_Error errorstatus   = SD_OK;
    uint32_t delay         = 0;
    __IO uint32_t maxdelay = 0;
    uint8_t cardstate      = 0;

    /*!< Check if the card coomnd class supports erase command */
    if (((CSD_Tab[1] >> 20) & SD_CCCC_ERASE) == 0)
    {
        errorstatus = SD_REQUEST_NOT_APPLICABLE;
        return (errorstatus);
    }

    // Note the delay here
    maxdelay = 120000 / ((SDIO->CLKCTRL & 0xFF) + 2); // Delay, calculated according to clock division setting

    if (SDIO_GetResp(SDIO_RESPONSE_1) & SD_CARD_LOCKED) // The card has been locked.
    {
        errorstatus = SD_LOCK_UNLOCK_FAILED;
        return (errorstatus);
    }

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD) // sdhc card, See 2.0 protocol page52 for details
    { // In SDHC card, the address parameter is block address, 512 bytes for each special block, and the address of SDSC
      // card is byte address
        startaddr /= 512;
        endaddr /= 512;
    }

    /*!< According to sd-card spec 1.0 ERASE_GROUP_START (CMD32) and erase_group_end(CMD33) */
    if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType)
        || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
    {
        /*!< Send CMD32 SD_ERASE_GRP_START with argument as addr  */
        SDIO_CmdInitStructure.CmdArgument  = startaddr;
        SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SD_ERASE_GRP_START;
        SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // R1
        SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
        SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
        SDIO_SendCmd(&SDIO_CmdInitStructure);

        errorstatus = CmdResp1Error(SD_CMD_SD_ERASE_GRP_START);
        if (errorstatus != SD_OK)
        {
            return (errorstatus);
        }

        /*!< Send CMD33 SD_ERASE_GRP_END with argument as addr  */
        SDIO_CmdInitStructure.CmdArgument  = endaddr;
        SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SD_ERASE_GRP_END;
        SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
        SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
        SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
        SDIO_SendCmd(&SDIO_CmdInitStructure);

        errorstatus = CmdResp1Error(SD_CMD_SD_ERASE_GRP_END);
        if (errorstatus != SD_OK)
        {
            return (errorstatus);
        }
    }

    if (SDIO_MULTIMEDIA_CARD == CardType)
    {
        /*!< Send CMD32 SD_ERASE_GRP_START with argument as addr    */
        SDIO_CmdInitStructure.CmdArgument  = startaddr;
        SDIO_CmdInitStructure.CmdIndex     = SD_CMD_ERASE_GRP_START;
        SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // R1
        SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
        SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
        SDIO_SendCmd(&SDIO_CmdInitStructure);

        errorstatus = CmdResp1Error(SD_CMD_ERASE_GRP_START);
        if (errorstatus != SD_OK)
        {
            return (errorstatus);
        }

        /*!< Send CMD33 SD_ERASE_GRP_END with argument as addr  */
        SDIO_CmdInitStructure.CmdArgument  = endaddr;
        SDIO_CmdInitStructure.CmdIndex     = SD_CMD_ERASE_GRP_END;
        SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
        SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
        SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
        SDIO_SendCmd(&SDIO_CmdInitStructure);

        errorstatus = CmdResp1Error(SD_CMD_ERASE_GRP_END);
        if (errorstatus != SD_OK)
        {
            return (errorstatus);
        }
    }
    /*!< Send CMD38 ERASE */
    SDIO_CmdInitStructure.CmdArgument  = 0;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_ERASE;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_ERASE);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    for (delay = 0; delay < maxdelay; delay++)
    {
    }

    /*!< Wait till the card is in programming state */
    errorstatus = IsCardProgramming(&cardstate);

    while ((errorstatus == SD_OK) && ((SD_CARD_PROGRAMMING == cardstate) || (SD_CARD_RECEIVING == cardstate)))
    {
        errorstatus = IsCardProgramming(&cardstate);
    }

    return (errorstatus);
}

/**
 * @brief  Returns the current card's status.
 * @param pcardstatus pointer to the buffer that will contain the SD card
 *         status (Card Status register).
 * @return SD_Error: SD Card Error code.
 */
SD_Error SD_SendStatus(uint32_t* pcardstatus)
{
    SD_Error errorstatus = SD_OK;

    SDIO->CMDARG  = (uint32_t)RCA << 16;
    SDIO->CMDCTRL = 0x44D;

    errorstatus = CmdResp1Error(SD_CMD_SEND_STATUS);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    *pcardstatus = SDIO->RESPONSE1;
    return (errorstatus);
}

/**
 * @brief  Returns the current SD card's status.
 * @param psdstatus pointer to the buffer that will contain the SD card status
 *         (SD Status register).
 * @return SD_Error: SD Card Error code.
 */
SD_Error SD_SendSDStatus(uint32_t* psdstatus)
{
    SD_Error errorstatus = SD_OK;
    uint32_t count       = 0;

    if (SDIO_GetResp(SDIO_RESPONSE_1) & SD_CARD_LOCKED)
    {
        errorstatus = SD_LOCK_UNLOCK_FAILED;
        return (errorstatus);
    }

    /*!< Set block size for card if it is not equal to current block size for card. */
    SDIO_CmdInitStructure.CmdArgument  = 64;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    /*!< CMD55 */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)RCA << 16;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_APP_CMD;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);
    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = 64;
    SDIO_DataInitStructure.DatBlkSize        = SDIO_DATBLK_SIZE_64B;
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOSDIO;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< Send ACMD13 SD_APP_STAUS  with argument as card's RCA.*/
    SDIO_CmdInitStructure.CmdArgument  = 0;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SD_APP_STAUS;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);
    errorstatus = CmdResp1Error(SD_CMD_SD_APP_STAUS);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    while (!(SDIO->STS
             & (SDIO_FLAG_RXORERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_DATBLKEND | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_RFIFOHF) != RESET)
        {
            for (count = 0; count < 8; count++)
            {
                *(psdstatus + count) = SDIO_ReadData();
            }
            psdstatus += 8;
        }
    }

    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_RXORERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_RXORERR);
        errorstatus = SD_RX_OVERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }

    while (SDIO_GetFlag(SDIO_FLAG_RDATVALID) != RESET)
    {
        *psdstatus = SDIO_ReadData();
        psdstatus++;
    }

    /*!< Clear all the static status flags*/
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);

    return (errorstatus);
}

/**
 * @brief  SDIO interrupt process function
 *
 * @return SD_Error: SD Card Error code.
 */
SD_Error SD_ProcessIRQSrc(void)
{
    if (StopCondition == 1)
    {
        SDIO->CMDARG  = 0x0;
        SDIO->CMDCTRL = 0x44C;
        TransferError = CmdResp1Error(SD_CMD_STOP_TRANSMISSION);
    }
    else
    {
        TransferError = SD_OK;
    }
    SDIO_ClrIntPendingBit(SDIO_INT_DATEND);
    SDIO_ConfigInt(SDIO_INT_DATEND, DISABLE);
    TransferEnd = 1;
    return (TransferError);
}

/**
 * @brief  Check the execution status of cmd0
 *
 * @return SD_Error: SD Card Error code.
 */
static SD_Error CmdError(void)
{
    SD_Error errorstatus = SD_OK;
    uint32_t timeout;

    timeout = SDIO_CMD0TIMEOUT; /*!< 10000 */
    while ((timeout > 0) && (SDIO_GetFlag(SDIO_FLAG_CMDSEND) == RESET))
    {
        timeout--;
    }

    if (timeout == 0)
    {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        return (errorstatus);
    }

    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);

    return (errorstatus);
}

/**
 * @brief  Check the error status of the R7 response
 *
 * @return SD_Error: SD Card Error code.
 */
static SD_Error CmdResp7Error(void)
{
    SD_Error errorstatus = SD_OK;
    uint32_t status;
    uint32_t timeout = SDIO_CMD0TIMEOUT;

    status = SDIO->STS;

    /* Command response received (CRC check failed) :Command response received (CRC check passed):Command response
     * timeout */

    while (!(status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDRESPRECV | SDIO_FLAG_CMDTIMEOUT)) && (timeout > 0))
    {
        timeout--;
        status = SDIO->STS;
    }
    if ((timeout == 0) || (status & SDIO_FLAG_CMDTIMEOUT))
    {
        /*!< Card is not V2.0 complient or card does not support the set voltage range */
        errorstatus = SD_CMD_RSP_TIMEOUT;
        SDIO_ClrFlag(SDIO_FLAG_CMDTIMEOUT);
        return (errorstatus);
    }

    if (status & SDIO_FLAG_CMDRESPRECV)
    {
        /*!< Card is SD V2.0 compliant */
        errorstatus = SD_OK;
        SDIO_ClrFlag(SDIO_FLAG_CMDRESPRECV);
        return (errorstatus);
    }
    return (errorstatus);
}

static SD_Error CmdResp1Error_ForIrqTest(uint8_t cmd)
{
    while (!(SDIO->STS & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDRESPRECV | SDIO_FLAG_CMDTIMEOUT)))
    {
    }
    // SDIO->INTCLR = SDIO_STATIC_FLAGS;
    // while(!(g_write_FLAG_CCRCFAIL | g_write_FLAG_CMDREND | g_write_FLAG_CTIMEOUT))
    {
    }
    return (SD_Error)(SDIO->RESPONSE1 & SD_OCR_ERRORBITS);
}

/**
 * @brief  Check error status of R1 response
 *
 * @return SD_Error: SD Card Error code.
 */
SD_Error CmdResp1Error(uint8_t cmd)
{
    while (!(SDIO->STS & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDRESPRECV | SDIO_FLAG_CMDTIMEOUT)))
    {
    }

    if (SDIO->STS & SDIO_FLAG_CMDTIMEOUT)
    {
        SDIO->INTCLR = SDIO_STATIC_FLAGS;
        return SD_CMD_RSP_TIMEOUT;
    }

    SDIO->INTCLR = SDIO_STATIC_FLAGS;
    return (SD_Error)(SDIO->RESPONSE1 & SD_OCR_ERRORBITS);
}

/**
 * @brief  Check error status of R3 response
 *
 * @return SD_Error: SD Card Error code.
 */
static SD_Error CmdResp3Error(void)
{
    SD_Error errorstatus = SD_OK;
    uint32_t status;

    status = SDIO->STS;

    while (!(status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDRESPRECV | SDIO_FLAG_CMDTIMEOUT)))
    {
        status = SDIO->STS;
    }

    if (status & SDIO_FLAG_CMDTIMEOUT)
    {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        SDIO_ClrFlag(SDIO_FLAG_CMDTIMEOUT);
        return (errorstatus);
    }
    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);
    return (errorstatus);
}

/**
 * @brief  Check error status of R2 response
 *
 * @return SD_Error: SD Card Error code.
 */
static SD_Error CmdResp2Error(void)
{
    SD_Error errorstatus = SD_OK;
    uint32_t status;

    status = SDIO->STS;

    while (!(status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTIMEOUT | SDIO_FLAG_CMDRESPRECV)))
    {
        status = SDIO->STS;
    }

    if (status & SDIO_FLAG_CMDTIMEOUT)
    {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        SDIO_ClrFlag(SDIO_FLAG_CMDTIMEOUT);
        return (errorstatus);
    }
    else if (status & SDIO_FLAG_CCRCERR)
    {
        errorstatus = SD_CMD_CRC_FAIL;
        SDIO_ClrFlag(SDIO_FLAG_CCRCERR);
        return (errorstatus);
    }

    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);

    return (errorstatus);
}

/**
 * @brief  Check error status of R6 response
 *
 * @return SD_Error: SD Card Error code.
 */
static SD_Error CmdResp6Error(uint8_t cmd, uint16_t* prca)
{
    SD_Error errorstatus = SD_OK;
    uint32_t status;
    uint32_t response_r1;

    status = SDIO->STS;

    while (!(status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTIMEOUT | SDIO_FLAG_CMDRESPRECV)))
    {
        status = SDIO->STS;
    }

    if (status & SDIO_FLAG_CMDTIMEOUT)
    {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        SDIO_ClrFlag(SDIO_FLAG_CMDTIMEOUT);
        return (errorstatus);
    }
    else if (status & SDIO_FLAG_CCRCERR)
    {
        errorstatus = SD_CMD_CRC_FAIL;
        SDIO_ClrFlag(SDIO_FLAG_CCRCERR);
        return (errorstatus);
    }

    /*!< Check response received is of desired command */
    if (SDIO_GetCmdResp() != cmd)
    {
        errorstatus = SD_ILLEGAL_CMD;
        return (errorstatus);
    }

    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);

    /*!< We have received response, retrieve it.  */
    response_r1 = SDIO_GetResp(SDIO_RESPONSE_1);

    if (SD_ALLZERO == (response_r1 & (SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_ILLEGAL_CMD | SD_R6_COM_CRC_FAILED)))
    {
        *prca = (uint16_t)(response_r1 >> 16);
        return (errorstatus);
    }

    if (response_r1 & SD_R6_GENERAL_UNKNOWN_ERROR)
    {
        return (SD_GENERAL_UNKNOWN_ERROR);
    }

    if (response_r1 & SD_R6_ILLEGAL_CMD)
    {
        return (SD_ILLEGAL_CMD);
    }

    if (response_r1 & SD_R6_COM_CRC_FAILED)
    {
        return (SD_COM_CRC_FAILED);
    }

    return (errorstatus);
}

/**
 * @brief  set bus wide
 * @param wideMode bus wide
 *
 * @return SD_Error: SD Card Error code.
 */
static SD_Error MMCSetWideBus(u32 WideMode)
{
    SD_Error errorstatus = SD_OK;

//    uint32_t scr[2] = {0, 0};

    if (SDIO_GetResp(SDIO_RESPONSE_1) & SD_CARD_LOCKED) // Check whether the card is locked
    {
        errorstatus = SD_LOCK_UNLOCK_FAILED;
        return (errorstatus);
    }
    // 4.0 or above to support cmd6
    if (((CSD_Tab[0] >> 30) & 0x3) < 2)
    {
        errorstatus = SD_UNSUPPORTED_FEATURE;
        return (errorstatus);
    }
    else if (((CSD_Tab[0] >> 30) & 0x3) == 2)
    {
        if (((CSD_Tab[0] >> 26) & 0xf) != 4)
        {
            errorstatus = SD_UNSUPPORTED_FEATURE;
            return (errorstatus);
        }
    }

    // send CMD6
    /*!< Send CMD6  with argument  for wide bus mode */
    /*acmd6: Command to turn on 4bit mode*/

    //?power class
    if (SDIO_BUSWIDTH_8B == WideMode)
    {
        SDIO_CmdInitStructure.CmdArgument = 0x03bb0800;
    }
    else if (SDIO_BUSWIDTH_4B == WideMode)
    {
        SDIO_CmdInitStructure.CmdArgument = 0x03bb0800;
    }
    else
    {
        SDIO_CmdInitStructure.CmdArgument = 0x03bb0800;
    }

    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_HS_SWITCH;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_HS_SWITCH);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }
    // check till not busy
    while (SD_GetStatus() != SD_TRANSFER_OK)
    {
        Delay_sdio_lock(0xffff);
    }

    // change bus wide
    if (SDIO_BUSWIDTH_8B == WideMode)
    {
        SDIO_CmdInitStructure.CmdArgument = 0x03b70200;
    }
    else if (SDIO_BUSWIDTH_4B == WideMode)
    {
        SDIO_CmdInitStructure.CmdArgument = 0x03b70100;
    }
    else
    {
        SDIO_CmdInitStructure.CmdArgument = 0x03b70000;
    }

    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_HS_SWITCH;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_HS_SWITCH);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    // check till not busy
    while (SD_GetStatus() != SD_TRANSFER_OK)
    {
        Delay_sdio_lock(0xffff);
    }

    return (errorstatus);
}

/**
 * @brief  Enable or disable 4 bit mode of SDIO
 * @param Cmd ENABLE or DISABLE
 *
 * @return SD_Error: SD Card Error code.
 */
static SD_Error SDEnWideBus(FunctionalState Cmd)
{
    SD_Error errorstatus = SD_OK;

    uint32_t scr[2] = {0, 0};

    if (SDIO_GetResp(SDIO_RESPONSE_1) & SD_CARD_LOCKED) // Check whether the card is locked
    {
        errorstatus = SD_LOCK_UNLOCK_FAILED;
        return (errorstatus);
    }

    /*!< Get SCR Register */
    errorstatus = FindSCR(RCA, scr); // Get SCR register contents to SCR array

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    /*!< If wide bus operation to be enabled */
    if (Cmd == ENABLE)
    {
        /*!< If requested card supports wide bus operation */
        if ((scr[1] & SD_WIDE_BUS_SUPPORT) != SD_ALLZERO) // whether the card supports 4-bit mode
        {
            /*!< Send CMD55 APP_CMD with argument as card's RCA.*/
            SDIO_CmdInitStructure.CmdArgument  = (uint32_t)RCA << 16;
            SDIO_CmdInitStructure.CmdIndex     = SD_CMD_APP_CMD;
            SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
            SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
            SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
            SDIO_SendCmd(&SDIO_CmdInitStructure);

            errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

            if (errorstatus != SD_OK)
            {
                return (errorstatus);
            }

            /*!< Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
            /*acmd6: Command to turn on 4bit mode*/
            SDIO_CmdInitStructure.CmdArgument  = 0x2;
            SDIO_CmdInitStructure.CmdIndex     = SD_CMD_APP_SD_SET_BUSWIDTH;
            SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
            SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
            SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
            SDIO_SendCmd(&SDIO_CmdInitStructure);

            errorstatus = CmdResp1Error(SD_CMD_APP_SD_SET_BUSWIDTH);

            if (errorstatus != SD_OK)
            {
                return (errorstatus);
            }
            return (errorstatus);
        }
        else
        {
            errorstatus = SD_REQUEST_NOT_APPLICABLE;
            return (errorstatus);
        }
    } /*!< If wide bus operation to be disabled */
    else
    {
        /*!< If requested card supports 1 bit mode operation */
        if ((scr[1] & SD_SINGLE_BUS_SUPPORT) != SD_ALLZERO)
        {
            /*!< Send CMD55 APP_CMD with argument as card's RCA.*/
            SDIO_CmdInitStructure.CmdArgument  = (uint32_t)RCA << 16;
            SDIO_CmdInitStructure.CmdIndex     = SD_CMD_APP_CMD;
            SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
            SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
            SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
            SDIO_SendCmd(&SDIO_CmdInitStructure);

            errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

            if (errorstatus != SD_OK)
            {
                return (errorstatus);
            }

            /*!< Send ACMD6 APP_CMD with argument as 0 for single bus mode */
            SDIO_CmdInitStructure.CmdArgument  = 0x00;
            SDIO_CmdInitStructure.CmdIndex     = SD_CMD_APP_SD_SET_BUSWIDTH;
            SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
            SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
            SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
            SDIO_SendCmd(&SDIO_CmdInitStructure);

            errorstatus = CmdResp1Error(SD_CMD_APP_SD_SET_BUSWIDTH);

            if (errorstatus != SD_OK)
            {
                return (errorstatus);
            }

            return (errorstatus);
        }
        else
        {
            errorstatus = SD_REQUEST_NOT_APPLICABLE;
            return (errorstatus);
        }
    }
}

/**
 * @brief  Check if SD card is in internal read-write operation
 * @param pstatus Pointer to load SD state state
 *
 * @return SD_Error: SD Card Error code.
 */
static SD_Error IsCardProgramming(uint8_t* pstatus)
{
    SD_Error errorstatus = SD_OK;
    __IO uint32_t respR1 = 0, status = 0;

    /*cmd13:Let the card send the card status register, and store it in m3 as SDIO? Sta register*/
    SDIO_CmdInitStructure.CmdArgument = (uint32_t)RCA << 16; // Card relative address parameters
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SEND_STATUS;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    status = SDIO->STS;
    while (!(status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDRESPRECV | SDIO_FLAG_CMDTIMEOUT)))
    {
        status = SDIO->STS;
    }

    if (status & SDIO_FLAG_CMDTIMEOUT)
    {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        SDIO_ClrFlag(SDIO_FLAG_CMDTIMEOUT);
        return (errorstatus);
    }
    else if (status & SDIO_FLAG_CCRCERR)
    {
        errorstatus = SD_CMD_CRC_FAIL;
        SDIO_ClrFlag(SDIO_FLAG_CCRCERR);
        return (errorstatus);
    }

    status = (uint32_t)SDIO_GetCmdResp();

    /*!< Check response received is of desired command */
    if (status != SD_CMD_SEND_STATUS)
    {
        errorstatus = SD_ILLEGAL_CMD;
        return (errorstatus);
    }

    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);

    /*!< We have received response, retrieve it for analysis  */
    respR1 = SDIO_GetResp(SDIO_RESPONSE_1);

    /*!< Find out card status */
    *pstatus = (uint8_t)((respR1 >> 9) & 0x0000000F); // status[12:9] :cardstate

    if ((respR1 & SD_OCR_ERRORBITS) == SD_ALLZERO)
    {
        return (errorstatus);
    }

    if (respR1 & SD_OCR_ADDR_OUT_OF_RANGE)
    {
        return (SD_ADDR_OUT_OF_RANGE);
    }

    if (respR1 & SD_OCR_ADDR_MISALIGNED)
    {
        return (SD_ADDR_MISALIGNED);
    }

    if (respR1 & SD_OCR_BLOCK_LEN_ERR)
    {
        return (SD_BLOCK_LEN_ERR);
    }

    if (respR1 & SD_OCR_ERASE_SEQ_ERR)
    {
        return (SD_ERASE_SEQ_ERR);
    }

    if (respR1 & SD_OCR_BAD_ERASE_PARAM)
    {
        return (SD_BAD_ERASE_PARAM);
    }

    if (respR1 & SD_OCR_WRITE_PROT_VIOLATION)
    {
        return (SD_WRITE_PROT_VIOLATION);
    }

    if (respR1 & SD_OCR_LOCK_UNLOCK_FAILED)
    {
        return (SD_LOCK_UNLOCK_FAILED);
    }

    if (respR1 & SD_OCR_COM_CRC_FAILED)
    {
        return (SD_COM_CRC_FAILED);
    }

    if (respR1 & SD_OCR_ILLEGAL_CMD)
    {
        return (SD_ILLEGAL_CMD);
    }

    if (respR1 & SD_OCR_CARD_ECC_FAILED)
    {
        return (SD_CARD_ECC_FAILED);
    }

    if (respR1 & SD_OCR_CC_ERROR)
    {
        return (SD_CC_ERROR);
    }

    if (respR1 & SD_OCR_GENERAL_UNKNOWN_ERROR)
    {
        return (SD_GENERAL_UNKNOWN_ERROR);
    }

    if (respR1 & SD_OCR_STREAM_READ_UNDERRUN)
    {
        return (SD_STREAM_READ_UNDERRUN);
    }

    if (respR1 & SD_OCR_STREAM_WRITE_OVERRUN)
    {
        return (SD_STREAM_WRITE_OVERRUN);
    }

    if (respR1 & SD_OCR_CID_CSD_OVERWRIETE)
    {
        return (SD_CID_CSD_OVERWRITE);
    }

    if (respR1 & SD_OCR_WP_ERASE_SKIP)
    {
        return (SD_WP_ERASE_SKIP);
    }

    if (respR1 & SD_OCR_CARD_ECC_DISABLED)
    {
        return (SD_CARD_ECC_DISABLED);
    }

    if (respR1 & SD_OCR_ERASE_RESET)
    {
        return (SD_ERASE_RESET);
    }

    if (respR1 & SD_OCR_AKE_SEQ_ERROR)
    {
        return (SD_AKE_SEQ_ERROR);
    }

    return (errorstatus);
}

SD_Error SD_ReadBlock_ForACMD13(uint8_t* readbuff, uint32_t ReadAddr, uint16_t BlockSize)
{
    SD_Error errorstatus = SD_OK;
#if defined(SD_POLLING_MODE)
    uint32_t count = 0, *tempbuff = (uint32_t*)readbuff;
#endif
    TransferError = SD_OK;
    TransferEnd   = 0;
    StopCondition = 0;

    SDIO->DATCTRL = 0x0;

    if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        ReadAddr /= 512;
    }

    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)BlockSize;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return (errorstatus);
    }

    /*!< Send CMD55 APP_CMD with argument as card's RCA */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)RCA << 16;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_APP_CMD;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    /*********************************************************************************/
    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = BlockSize;
    SDIO_DataInitStructure.DatBlkSize        = (uint32_t)9 << 4;
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOSDIO;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< Send ACMD13 READ_SINGLE_BLOCK */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)0;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SD_APP_STAUS;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SD_APP_STAUS);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

#if defined(SD_POLLING_MODE)
    /*!< In case of single block transfer, no need of stop transfer at all.*/
    /*!< Polling mode */
    while (!(SDIO->STS
             & (SDIO_FLAG_RXORERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_DATBLKEND | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_RFIFOHF) != RESET)
        {
            for (count = 0; count < 8; count++)
            {
                *(tempbuff + count) = SDIO_ReadData();
            }
            tempbuff += 8;
        }
    }

    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_RXORERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_RXORERR);
        errorstatus = SD_RX_OVERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }

    while (SDIO_GetFlag(SDIO_FLAG_RDATVALID) != RESET)
    {
        *tempbuff = SDIO_ReadData();
        tempbuff++;
    }
    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);
#elif defined(SD_DMA_MODE)
    SDIO_ConfigInt(SDIO_INT_DATEND, ENABLE);
    SDIO_DMACmd(ENABLE);
    SD_DMA_RxConfig((uint32_t*)readbuff, BlockSize);
#endif

    return (errorstatus);
}

SD_Error SD_GetACMD22(uint32_t* pscr)
{
//    uint16_t rca         = RCA;
    uint32_t index       = 0;
    SD_Error errorstatus = SD_OK;
    uint32_t tempscr[2]  = {0, 0};

    /*!< Set Block Size To 8 Bytes */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)4;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    /*!< Send CMD55 APP_CMD with argument as card's RCA */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)RCA << 16;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_APP_CMD;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = 4; // 8byte,64?
    SDIO_DataInitStructure.DatBlkSize        = SDIO_DATBLK_SIZE_4B;
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOSDIO;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< Send ACMD51 SD_APP_SEND_SCR with argument as 0 */
    SDIO_CmdInitStructure.CmdArgument  = 0x0;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SD_APP_SEND_NUM_WRITE_BLOCKS;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SD_APP_SEND_NUM_WRITE_BLOCKS);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    while (!(SDIO->STS
             & (SDIO_FLAG_RXORERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_DATBLKEND | SDIO_FLAG_SBERR)))
    {
        if (SDIO_GetFlag(SDIO_FLAG_RDATVALID) != RESET)
        {
            *(tempscr + index) = SDIO_ReadData();
            index++;

            if (index >= 1)
                break;
        }
    }

    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_RXORERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_RXORERR);
        errorstatus = SD_RX_OVERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }

    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);
    *(pscr) = tempscr[0];
    //*(pscr + 1) = ((tempscr[0] & SD_0TO7BITS) << 24) | ((tempscr[0] & SD_8TO15BITS) << 8) | ((tempscr[0] &
    //SD_16TO23BITS) >> 8) | ((tempscr[0] & SD_24TO31BITS) >> 24);
    //*(pscr) = ((tempscr[1] & SD_0TO7BITS) << 24) | ((tempscr[1] & SD_8TO15BITS) << 8) | ((tempscr[1] & SD_16TO23BITS)
    //>> 8) | ((tempscr[1] & SD_24TO31BITS) >> 24);

    return (errorstatus);
}

/**
 * @brief  Read the contents of SCR register of SD card
 * @param rca RCA card relative address
 *         pscr: Pointer to load SCR content
 * @return SD_Error: SD Card Error code.
 */
static SD_Error FindSCR(uint16_t rca, uint32_t* pscr)
{
    uint32_t index       = 0;
    SD_Error errorstatus = SD_OK;
    uint32_t tempscr[2]  = {0, 0};

    /*!< Set Block Size To 8 Bytes */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)8;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SET_BLOCKLEN; //     cmd16
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;     // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    /*!< Send CMD55 APP_CMD with argument as card's RCA */
    SDIO_CmdInitStructure.CmdArgument  = (uint32_t)RCA << 16;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_APP_CMD;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT;
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    SDIO_DataInitStructure.DatTimeout        = SD_DATATIMEOUT;
    SDIO_DataInitStructure.DatLen            = 8;                   // 8byte,64?
    SDIO_DataInitStructure.DatBlkSize        = SDIO_DATBLK_SIZE_8B; //???8byte
    SDIO_DataInitStructure.TransferDirection = SDIO_TRANSDIR_TOSDIO;
    SDIO_DataInitStructure.TransferMode      = SDIO_TRANSMODE_BLOCK;
    SDIO_DataInitStructure.DPSMConfig        = SDIO_DPSM_ENABLE;
    SDIO_ConfigData(&SDIO_DataInitStructure);

    /*!< Send ACMD51 SD_APP_SEND_SCR with argument as 0 */
    SDIO_CmdInitStructure.CmdArgument  = 0x0;
    SDIO_CmdInitStructure.CmdIndex     = SD_CMD_SD_APP_SEND_SCR;
    SDIO_CmdInitStructure.ResponseType = SDIO_RESP_SHORT; // r1
    SDIO_CmdInitStructure.WaitType     = SDIO_WAIT_NO;
    SDIO_CmdInitStructure.CPSMConfig   = SDIO_CPSM_ENABLE;
    SDIO_SendCmd(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SD_APP_SEND_SCR);

    if (errorstatus != SD_OK)
    {
        return (errorstatus);
    }

    // while (!(SDIO->STS & (SDIO_FLAG_RXORERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_DATBLKEND|
    // SDIO_FLAG_SBERR)))
    while (!(SDIO->STS & (SDIO_FLAG_RXORERR | SDIO_FLAG_DCRCERR | SDIO_FLAG_DATTIMEOUT | SDIO_FLAG_SBERR)))
    {
        // if (SDIO_GetFlag(SDIO_FLAG_RDATVALID) != RESET)
        if ((SDIO->STS & SDIO_FLAG_RDATVALID) != (uint32_t)RESET)
        {
            *(tempscr + index) = SDIO_ReadData();
            index++;

            if (index > 1)
                break;
        }
    }

    if (SDIO_GetFlag(SDIO_FLAG_DATTIMEOUT) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DATTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_DCRCERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_DCRCERR);
        errorstatus = SD_DATA_CRC_FAIL;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_RXORERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_RXORERR);
        errorstatus = SD_RX_OVERRUN;
        return (errorstatus);
    }
    else if (SDIO_GetFlag(SDIO_FLAG_SBERR) != RESET)
    {
        SDIO_ClrFlag(SDIO_FLAG_SBERR);
        errorstatus = SD_START_BIT_ERR;
        return (errorstatus);
    }

    /*!< Clear all the static flags */
    SDIO_ClrFlag(SDIO_STATIC_FLAGS);

    *(pscr + 1) = ((tempscr[0] & SD_0TO7BITS) << 24) | ((tempscr[0] & SD_8TO15BITS) << 8)
                  | ((tempscr[0] & SD_16TO23BITS) >> 8) | ((tempscr[0] & SD_24TO31BITS) >> 24);

    *(pscr) = ((tempscr[1] & SD_0TO7BITS) << 24) | ((tempscr[1] & SD_8TO15BITS) << 8)
              | ((tempscr[1] & SD_16TO23BITS) >> 8) | ((tempscr[1] & SD_24TO31BITS) >> 24);

    return (errorstatus);
}

/**
 * @brief  Converts the number of bytes in power of two and returns the power.
 * @param NumberOfBytes number of bytes.
 */
uint8_t convert_from_bytes_to_power_of_two(uint16_t NumberOfBytes)
{
    uint8_t count = 0;

    while (NumberOfBytes != 1)
    {
        NumberOfBytes >>= 1;
        count++;
    }
    return (count);
}

/**
 * @brief  Read SD card.
 * @param buf write buf
 *         sector: sector addr
 *         cnt: sector count
 * @return Error status;
 *         0, normal; other, error code;
 */
u8 SD_ReadDisk(u8* buf, u32 sector, u8 cnt)
{
    u8 sta            = SD_OK;
    long long lsector = sector;
    u8 n;
    lsector <<= 9;
    if ((u32)buf % 4 != 0)
    {
        for (n = 0; n < cnt; n++)
        {
            sta = SD_ReadBlock(SDIO_DATA_BUFFER, lsector + 512 * n, 512); // Read operation of a single sector
            memcpy(buf, SDIO_DATA_BUFFER, 512);
            buf += 512;
        }
    }
    else
    {
        if (cnt == 1)
            sta = SD_ReadBlock(buf, lsector, 512); // Read operation of a single sector
        else
            sta = SD_ReadMultiBlocks(buf, lsector, 512, cnt); // Multiple sector
    }
    return sta;
}

/**
 * @brief  Write SD card.
 * @param buf write buf
 *         sector: sector addr
 *         cnt: sector count
 * @return Error status;
 *         0, normal; other, error code;
 */
u8 SD_WriteDisk(u8* buf, u32 sector, u8 cnt)
{
    u8 sta = SD_OK;
    u8 n;
    long long lsector = sector;
    lsector <<= 9;
    if ((u32)buf % 4 != 0)
    {
        for (n = 0; n < cnt; n++)
        {
            memcpy(SDIO_DATA_BUFFER, buf, 512);
            sta = SD_WriteBlock(SDIO_DATA_BUFFER, lsector + 512 * n, 512); // Write operation of a single sector
            buf += 512;
        }
    }
    else
    {
        if (cnt == 1)
            sta = SD_WriteBlock(buf, lsector, 512); // Write operation of a single sector
        else
            sta = SD_WriteMultiBlocks(buf, lsector, 512, cnt); // Multiple sector
    }
    return sta;
}

void SDIO_IRQHandler(void)
{
    SD_ProcessIRQSrc();
}

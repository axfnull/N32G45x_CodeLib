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
 * @file main.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32g45x.h"

/*@brief W25Q128FV instruction*/

/**
* @brief  W25Q128FV instruction

*/

#include <stdint.h>

#if 0
#define QSPI_OK          1


/* Reset */
#define RESET_ENABLE_CMD 0x66

#define RESET_MEMORY_CMD 0x99

#define ENTER_QPI_MODE_CMD 0x38

#define EXIT_QPI_MODE_CMD 0xFF

/* Recongize */

#define READ_ID_CMD 0x90

#define DUAL_READ_ID_CMD 0x92

#define QUAD_READ_ID_CMD 0x94

#define READ_JEDEC_ID_CMD 0x9F

/* Read */

#define READ_CMD 0x03

#define FAST_READ_CMD 0x0B

#define DUAL_OUT_FAST_READ_CMD 0x3B

#define DUAL_INOUT_FAST_READ_CMD 0xBB

#define QUAD_OUT_FAST_READ_CMD 0x6B

#define QUAD_INOUT_FAST_READ_CMD 0xEB

/* Write */

#define WRITE_ENABLE_CMD 0x06

#define WRITE_DISABLE_CMD 0x04

/* Register */

#define READ_STATUS_REG1_CMD 0x05

#define READ_STATUS_REG2_CMD 0x35

#define READ_STATUS_REG3_CMD 0x15

#define WRITE_STATUS_REG1_CMD 0x01

#define WRITE_STATUS_REG2_CMD 0x31

#define WRITE_STATUS_REG3_CMD 0x11

/* Program */

#define PAGE_PROG_CMD 0x02

#define QUAD_INPUT_PAGE_PROG_CMD 0x32

#define EXT_QUAD_IN_FAST_PROG_CMD 0x12

/* Erase */

#define SECTOR_ERASE_CMD 0x20

#define CHIP_ERASE_CMD 0xC7

#define PROG_ERASE_RESUME_CMD 0x7A

#define PROG_ERASE_SUSPEND_CMD 0x75

#endif

//#define QSPI_SAMPLE_SHIFTING_NONE           ((uint32_t)0x00000000U)        /*!<No clock cycle shift to sample data*/
//#define QSPI_SAMPLE_SHIFTING_HALFCYCLE      ((uint32_t)QUADSPI_CR_SSHIFT) /*!<1/2 clock cycle shift to sample data*/

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

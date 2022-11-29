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
 * @file usb_endp.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_istr.h"
#include "vs10xx.h"
#include "hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t m_tx_flag1;
extern uint8_t m_tx_flag2;
extern uint8_t m_tx_buf1[AUDIO_DATA_LENGTH];
extern uint8_t m_tx_buf2[AUDIO_DATA_LENGTH];

uint8_t Stream_Buff[32]={127,176,218,245,255,245,218,176,127,78,36,9,0,9,36,78,127,176,218,245,255,245,218,176,127,78,36,9,0,9,36,78};    //Sine WAVE

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief Endpoint 1 in callback routine.
 */
void EP1_IN_Callback(void)
{
    if (_GetENDPOINT(ENDP1) & EP_DATTOG_RX)
    {
        if(m_tx_flag1)
        {
            USB_FreeUserBuf(ENDP1, EP_DBUF_IN);
            USB_CopyUserToPMABuf(m_tx_buf1, ENDP1_BUF0Addr, AUDIO_DATA_LENGTH);
            USB_SetEpDblBuf0Cnt(ENDP1, EP_DBUF_IN, AUDIO_DATA_LENGTH);
            m_tx_flag1 = 0;
        }
    }
    else
    {
        if(m_tx_flag2)
        {
            USB_FreeUserBuf(ENDP1, EP_DBUF_IN);
            USB_CopyUserToPMABuf(m_tx_buf2, ENDP1_BUF1Addr, AUDIO_DATA_LENGTH);
            USB_SetEpDblBuf1Cnt(ENDP1, EP_DBUF_IN, AUDIO_DATA_LENGTH);
            m_tx_flag2 = 0;
        }
    }
}


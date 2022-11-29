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
 * @file recorder.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include "recorder.h"
#include "vs10xx.h"
#include "stdio.h"
#include "delay.h"
#include "hw_config.h"

uint8_t m_tx_flag1 = 0;
uint8_t m_tx_flag2 = 0;
uint8_t m_tx_buf1[AUDIO_DATA_LENGTH];
uint8_t m_tx_buf2[AUDIO_DATA_LENGTH];


void erase_recoder_file_in_flash(void);
//VS1053 WAV recoder has a bug,this plugin can correction this bug
const uint16_t wav_plugin[40]=/* Compressed plugin */ 
{ 
    0x0007, 0x0001, 0x8010, 0x0006, 0x001c, 0x3e12, 0xb817, 0x3e14, /* 0 */ 
    0xf812, 0x3e01, 0xb811, 0x0007, 0x9717, 0x0020, 0xffd2, 0x0030, /* 8 */ 
    0x11d1, 0x3111, 0x8024, 0x3704, 0xc024, 0x3b81, 0x8024, 0x3101, /* 10 */ 
    0x8024, 0x3b81, 0x8024, 0x3f04, 0xc024, 0x2808, 0x4800, 0x36f1, /* 18 */ 
    0x9811, 0x0007, 0x0001, 0x8028, 0x0006, 0x0002, 0x2a00, 0x040e,  
}; 

/**
 * @brief  enter pcm recoder mode
 * @param  agc : gain
 */
void recoder_enter_rec_mode(uint16_t agc)
{
    while(VS_DQ==0);

    VS_WR_Cmd(SPI_BASS,0x0000);    
    VS_WR_Cmd(SPI_AICTRL0,16000);    //set sampling rate 16Khz
    VS_WR_Cmd(SPI_AICTRL1,agc);     //set agc:0,default gain.1024 equal to 1,512 equal to 0.5, max value 65535=64
    VS_WR_Cmd(SPI_AICTRL2,0);       //set gain max,0,  65536=64X
    VS_WR_Cmd(SPI_AICTRL3,6);       //MIC 6:single left channel input PCM  5:double channel input PCM
    VS_WR_Cmd(SPI_CLOCKF,0x2000);   //set vs10xx clock,MULT:2 doube frequency;ADD:not allow;CLK:12.288Mhz
    VS_WR_Cmd(SPI_MODE,0x1804);     //MIC,recoder active   
    delay_ms(5);                    //wait at least 1.35ms 
    VS_Load_Patch((uint16_t*)wav_plugin,40);//VS1053 WAV recoder need patch
}
 
/**
 * @brief  recoder play 
 */
void recoder_play(void)
{
    uint16_t w;
    uint16_t idx=0;
    uint8_t recagc=4;   //default gain

    while(VS_HD_Reset());
    VS_Soft_Reset();
    recoder_enter_rec_mode(recagc*1024);
    while((VS_RD_Reg(SPI_HDAT1)>>8));
    
    while(1)
    {
        //read data from vs1053b
        w=VS_RD_Reg(SPI_HDAT1);     //get vs1053b buf exit data count
        if((w>=256 && w<896))      //256¡¢896 &&(w<896)
        {
            idx=0;
            while(idx<2*AUDIO_DATA_LENGTH)
            {
                w=VS_RD_Reg(SPI_HDAT0);     //get data
                if(idx<AUDIO_DATA_LENGTH && !m_tx_flag1)
                {
                    m_tx_buf1[idx++]=w&0XFF;
                    m_tx_buf1[idx++]=w>>8;
                }
                else if(idx>=AUDIO_DATA_LENGTH && idx <2*AUDIO_DATA_LENGTH && !m_tx_flag2)
                {
                    m_tx_buf2[idx++ - AUDIO_DATA_LENGTH]=w&0XFF;
                    m_tx_buf2[idx++ - AUDIO_DATA_LENGTH]=w>>8;
                }
                
                if(idx == AUDIO_DATA_LENGTH && !m_tx_flag1)
                {
                    m_tx_flag1 = 1;
                }
                if(idx == 2*AUDIO_DATA_LENGTH && !m_tx_flag2)
                {
                    m_tx_flag2 = 1;
                }
            }
        }
    }
}

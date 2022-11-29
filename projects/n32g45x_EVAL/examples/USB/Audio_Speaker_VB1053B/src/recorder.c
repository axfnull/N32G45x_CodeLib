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

#define RECORDERADDR    (4*1024*1024)
#define DATASIZE        512


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
* @brief  recoder wav init
 */
void recoder_wav_init(__WaveHeader* wavhead)   
{
    wavhead->riff.ChunkID=0X46464952;   //"RIFF"
    wavhead->riff.ChunkSize=0xFFFFFFFF; //size
    wavhead->riff.Format=0X45564157;    //"WAVE"
    wavhead->fmt.ChunkID=0X20746D66;    //"fmt "
    wavhead->fmt.ChunkSize=16;          //16bit
    wavhead->fmt.AudioFormat=0X01;      //0X01,PCM;0X01,IMA ADPCM
    wavhead->fmt.NumOfChannels=1;       //single channel
    wavhead->fmt.SampleRate=22050;      //22Khz sampling rate
    wavhead->fmt.ByteRate=wavhead->fmt.SampleRate;//16bit 2byte
    wavhead->fmt.BlockAlign=1;          //block size,1 block = 2byte
    wavhead->fmt.BitsPerSample=8;       //8 bit PCM
    wavhead->data.ChunkID=0X61746164;   //"data"
    wavhead->data.ChunkSize=0;          //data size
}
 
/**
* @brief  play wav file 
* @return play result 
 */
void rec_play_wav(__WaveHeader *wavhead)
{
    uint8_t *databuf;
    uint16_t i=0;
    uint16_t data_size = 0;
    
    while(VS_HD_Reset());       //VS hardware reset
    VS_Soft_Reset();            //VS software reset
    VS_Restart_Play();
    VS_Set_All();
    VS_Reset_DecodeTime();      //reset decode time
    VS_Load_Patch((uint16_t*)wav_plugin,40);//VS1053 WAV recoder need patch
     
    databuf = (uint8_t *)wavhead;
    data_size = sizeof(__WaveHeader);

    while(1)
    {
        VS_XDCS = 1;
        i=0;
        do
        {
            if(VS_Send_MusicData(databuf+i)==0)
            {
                i+=AUDIO_DATA_LENGTH;//send music data to vs
            }
        }while(i<data_size);//send 512 byte        
        VS_XDCS = 0;
        break;
    }
}

/**
* @brief  recoder play 
* @return play result 
 */
void recoder_play(void)
{
    __WaveHeader wavhead;

    while(VS_HD_Reset());
    VS_Soft_Reset();
    while((VS_RD_Reg(SPI_HDAT1)>>8));
    recoder_wav_init(&wavhead);//init WAV data
    
    rec_play_wav(&wavhead);
}

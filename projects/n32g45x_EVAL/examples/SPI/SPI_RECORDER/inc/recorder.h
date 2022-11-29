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
 * @file recorder.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef __RECORDER_H
#define __RECORDER_H
#include "stdint.h"

 //RIFF block
typedef __packed struct
{
    uint32_t ChunkID;           //chunk id;it is "RIFF",0X46464952
    uint32_t ChunkSize ;        //chunksize;filesize-8
    uint32_t Format;            //format;WAVE,0X45564157
}ChunkRIFF ;

//fmt block
typedef __packed struct
{
    uint32_t ChunkID;               //chunk id;it is "fmt ",0X20746D66
    uint32_t ChunkSize ;            //chunksize(not include ID and Size);it is 20.
    uint16_t AudioFormat;           //Audio Format;0X10,PCM;0X11,IMA ADPCM
    uint16_t NumOfChannels;         //number of channel ;1,single track;2,double track
    uint32_t SampleRate;            //Sample Rate;0X1F40,8Khz
    uint32_t ByteRate;              //Byte Rate
    uint16_t BlockAlign;            //Block Align
    uint16_t BitsPerSample;         //Single sample data size;4 bit ADPCM,set 4
}ChunkFMT;

//fact block
typedef __packed struct 
{
    uint32_t ChunkID;           //chunk id;it is"fact",0X74636166;
    uint32_t ChunkSize ;        //chunksize(not include ID and Size);it is 4
    uint32_t NumOfSamples;      //number of samples; 
}ChunkFACT;

//data block
typedef __packed struct 
{
    uint32_t ChunkID;           //chunk id;it is"data",0X61746164
    uint32_t ChunkSize ;        //chunksize(not include ID and Size);file size -60.
}ChunkDATA;

//wav header
typedef __packed struct
{ 
    ChunkRIFF riff; //riff block
    ChunkFMT fmt;   //fmt block
    ChunkDATA data; //data block 
}__WaveHeader; 
            
void recoder_enter_rec_mode(uint16_t agc);
void recoder_wav_init(__WaveHeader* wavhead);//wav init
void recoder_show_agc(uint8_t agc);        
void recoder_show_time(uint32_t tsec);
uint8_t recoder_play(void);                                      
#endif

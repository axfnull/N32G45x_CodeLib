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
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include "recorder.h"    
#include "vs10xx.h"   
#include "key.h"
#include "spi_flash.h"
#include "malloc.h"
#include "stdio.h"
#include "delay.h"

#define RECORDERADDR    (4*1024*1024)
#define DATASIZE  512


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
 */
void recoder_enter_rec_mode(uint16_t agc)
{
    while(VS_DQ==0);
    
    VS_WR_Cmd(SPI_BASS,0x0000);    
    VS_WR_Cmd(SPI_AICTRL0,8000);    //set sampling rate 8Khz
    VS_WR_Cmd(SPI_AICTRL1,agc);     //set agc:0,default gain.1024 equal to 1,512 equal to 0.5, max value 65535=64
    VS_WR_Cmd(SPI_AICTRL2,0);           //set gain max,0,  65536=64X
    VS_WR_Cmd(SPI_AICTRL3,6);           //MIC single left channel input PCM
    VS_WR_Cmd(SPI_CLOCKF,0x2000);   //set vs10xx clock,MULT:2 doube frequency;ADD:not allow;CLK:12.288Mhz
    VS_WR_Cmd(SPI_MODE,0x1804);     //MIC,recoder active   
    delay_ms(5);                                    //wait at least 1.35ms 
    VS_Load_Patch((uint16_t*)wav_plugin,40);//VS1053 WAV recoder need patch
}

/**
* @brief  recoder wav init
 */
void recoder_wav_init(__WaveHeader* wavhead)   
{
    wavhead->riff.ChunkID=0X46464952;   //"RIFF"
    wavhead->riff.ChunkSize=0;          //size
    wavhead->riff.Format=0X45564157;    //"WAVE"
    wavhead->fmt.ChunkID=0X20746D66;    //"fmt "
    wavhead->fmt.ChunkSize=16;          //16bit
    wavhead->fmt.AudioFormat=0X01;      //0X01,PCM;0X01,IMA ADPCM
    wavhead->fmt.NumOfChannels=1;       //single channel
    wavhead->fmt.SampleRate=8000;       //8Khz sampling rate
    wavhead->fmt.ByteRate=wavhead->fmt.SampleRate*2;//16bit 2byte
    wavhead->fmt.BlockAlign=2;          //block size,1 block = 2byte
    wavhead->fmt.BitsPerSample=16;      //16 bit PCM
    wavhead->data.ChunkID=0X61746164;   //"data"
    wavhead->data.ChunkSize=0;          //data size
}      
 
/**
* @brief  play wav file 
* @return play result 
 */
uint8_t rec_play_wav()
{
    uint8_t rval=0;
    uint8_t *databuf;
    uint16_t i=0;
    uint32_t sectorsize=0,n=0;
    uint16_t data_size = 512;
    
    databuf = mymalloc(data_size);
    if(databuf==NULL)rval=0XFF;//malloc memory fail

    if(rval==0)
    {
        VS_HD_Reset();          //VS hardware reset
        VS_Soft_Reset();            //VS software reset
        VS_Set_Vol(220);            //set the volume
        VS_Reset_DecodeTime();  //reset decode time
         
        //read back file size£¬file size-8
        sFLASH_ReadBuffer(databuf,RECORDERADDR-44+4,4);
        sectorsize+=databuf[3]<<24;
        sectorsize+=databuf[2]<<16;
        sectorsize+=databuf[1]<<8;
        sectorsize+=databuf[0];
        if(sectorsize == 0xFFFFFFFF)
        {
            printf("no recoder file\r\n");
            return 1;
        }
        sectorsize+=8;//file size
        VS_SPI_SpeedHigh();
        while(1)
        {
            VS_XDCS = 1;
            sFLASH_ReadBuffer(databuf,RECORDERADDR-44+n*data_size,data_size);
            i=0;
            do
            {   
                if(VS_Send_MusicData(databuf+i)==0)
                {
                    i+=32;//send music data to vs
                }
            }while(i<data_size);//send 512 byte
            n++;
            if(n%2 == 0)
            {
                printf("play=%d\r\n",n/2);
            }
            if(n==((sectorsize-44)/data_size)+1)
            {
                rval=0;
                printf("the file size=%dkByte\r\n",sectorsize/1024);
                break;
            }
            VS_XDCS = 0;
        }
    }
    myfree(databuf);
    return rval;
}    

/**
* @brief  recoder play 
* @return play result 
 */
uint8_t recoder_play(void)
{
    uint8_t key;
    uint8_t rval=0;
    __WaveHeader *wavhead=0;
    uint32_t sectorsize=0;
    uint8_t *recbuf = {0};  
    uint16_t w;
    uint16_t idx=0;     
    uint8_t rec_sta=0;//recoder state
                      //[7]:0,no recoder;1,exit recoder;
                      //[6:1]:reserve
                      //[0]:0,recoding;1,pause recode

    uint8_t recagc=4;   //default gain
    uint8_t playFlag=0; //recoder play flag
    
    wavhead=(__WaveHeader*)mymalloc(sizeof(__WaveHeader));
    if(wavhead==NULL)rval=1; 
    
    recbuf=mymalloc(DATASIZE);  
    if(recbuf==NULL)rval=1; 
    
    if(rval==0)
    {   
        while(VS_RD_Reg(SPI_HDAT1)>>8);         //wait buf free
        printf("Record test!");
        while(rval==0)
        {
            key=KEY_Scan(0);
            switch(key)
            {
                case KEY0_PRES: //REC/PAUSE
                    printf("key0:rec is down\r\n");
                    if(rec_sta&0X01)
                    {
                        rec_sta&=0XFE;//pausing, cancel pause
                    }
                    else if(rec_sta&0X80)//recording,pause
                    {
                        rec_sta|=0X01;  //pause
                    }
                    else                //no recording
                    {
                        printf("erase flash,please wait a moment\r\n");
                        erase_recoder_file_in_flash();  
                        printf("start rec\r\n");
                        while(VS_HD_Reset());
                        VS_Soft_Reset();
                        recoder_enter_rec_mode(recagc*1024);                
                        while((VS_RD_Reg(SPI_HDAT1)>>8));
                        rec_sta|=0X80;  //start recode 
                        recoder_wav_init(wavhead);//init WAV data
                        delay_ms(200);
                    }
                    break;
                    
                case KEY1_PRES: //STOP&SAVE
                    printf("key1:stop/save is down\r\n");
                    if(rec_sta&0X80)//exit record data
                    {
                        wavhead->riff.ChunkSize=sectorsize*DATASIZE+36;     //all file size
                        wavhead->data.ChunkSize=sectorsize*DATASIZE;        //data size
                        W25QXX_Write((uint8_t*)wavhead, RECORDERADDR-44, sizeof(__WaveHeader));//write WaveHeader data 44Byte
                        printf("save file in flash ok!all file size=%dkByte\r\n",(sectorsize*DATASIZE+44)/1024);
                        sectorsize=0;
                    }
                    rec_sta=0;
                    break;   
                    
                case KEY2_PRES://play record
                    printf("wk_up:play is down\r\n");
                    if(rec_sta==0)
                    {
                        playFlag=1;
                    }
                    break;
            } 
            
            //read data from vs1053b
            if(rec_sta==0X80)
            {
                w=VS_RD_Reg(SPI_HDAT1); //get vs1053b buf exit data count
                if((w>=256 && w<=896))
                {
                    idx=0;  
                    while(idx<DATASIZE)
                    {
                        w=VS_RD_Reg(SPI_HDAT0); //get data
                        recbuf[idx++]=w&0XFF;
                        recbuf[idx++]=w>>8;
                    }
                    if(sectorsize<1024*1024/DATASIZE)//recoder max size 1M
                    {
                        sFLASH_WriteBuffer(recbuf,RECORDERADDR+sectorsize*DATASIZE,DATASIZE);//write data to flash 512 byte
                        sectorsize++;
                    }
                    else
                    {
                        sectorsize=1024*1024/DATASIZE;
                        printf("err:flash is all,sectorsize=%d\r\n",sectorsize);        
                    }
                }
            }
            else
            {
                if(playFlag)
                {
                    printf("play:\r\n");
                    rec_play_wav();
                    playFlag = 0;
                    while(VS_HD_Reset());
                }
                delay_ms(5);
            }               
        }       
    }       
    myfree(wavhead);
    myfree(recbuf);
    return rval;
}

void erase_recoder_file_in_flash()
{
    uint8_t data_buf[4] = {0};
    uint8_t i = 0;
    uint32_t recoder_data_size = 0;
    printf("erase flash wait a moment\r\n");
    sFLASH_ReadBuffer(data_buf,RECORDERADDR-44+4,4);
    recoder_data_size+=data_buf[3]<<24;
    recoder_data_size+=data_buf[2]<<16;
    recoder_data_size+=data_buf[1]<<8;
    recoder_data_size+=data_buf[0];
    if(recoder_data_size > 1024*1024) recoder_data_size = 1024*1024;
    for(i=0;i<recoder_data_size/1024/64 + 1;i++)//16*64= 1024
    {
        W25QXX_Erase_Block(RECORDERADDR + i*1024*64);   // erase block  1block = 64k
    }
    W25QXX_Erase_Sector(RECORDERADDR - 4*1024);
    printf("erase flash succeed\r\n");
}


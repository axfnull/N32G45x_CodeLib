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
 * @file vs10xx.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include "vs10xx.h"
#include "spi.h"
#include "core_cm4.h"
#include <stdio.h>
#include "delay.h"
#include "hw_config.h"

//VS10XX default set param
_vs10xx_obj vsset=
{
    220,    //vol:220
    6,      //upper limit of the bass 60Hz
    15,     //bass boost 15dB
    10,     //high limit 10Khz
    15,     //bigh boost 10.5dB
    0,      //space effect
};


/**
 * @brief  Sends a byte through the SPI interface and return the byte received
 *         from the SPI bus.
 * @param  data to send.
 * @return The value of the received byte.
 */
uint8_t VS_SPI_ReadWriteByte(uint8_t data)
{
    return SPI3_ReadWriteByte(data);
}

/**
 * @brief  set vs spi speed low
 */
void VS_SPI_SpeedLow(void)
{
    SPI3_SetSpeed(SPI_BR_PRESCALER_16);
}

/**
 * @brief  set vs spi speed high
 */
void VS_SPI_SpeedHigh(void)
{
    SPI3_SetSpeed(SPI_BR_PRESCALER_4);
}

/**
 * @brief  vs1053b init 
 */
void VS_Init(void)
{
    GPIO_InitType  GPIO_InitStructure;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_GPIOC, ENABLE);

    GPIO_InitStructure.Pin = VS_DREQ_PIN;           //PA4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(VS_DREQ_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = VS_RST_PIN;            //PB0
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(VS_RST_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = VS_XCS_PIN;            //PC2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(VS_XCS_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = VS_XDCS_PIN;//PA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(VS_XDCS_PORT, &GPIO_InitStructure);

    SPI3_Init();
}

/**
 * @brief  vs1053b soft reset
 */
void VS_Soft_Reset(void)
{
    uint8_t retry=0;
    while(VS_DQ==0);                //wait soft reset finish  
    VS_SPI_ReadWriteByte(0Xff);
    retry=0;
    while(VS_RD_Reg(SPI_MODE)!=0x0800)
    {
        VS_WR_Cmd(SPI_MODE,0x0804); //soft reset
        systick_delay_ms(2);//wait at least 1.35ms 
        if(retry++>100)break;
    }
    while(VS_DQ==0);//wait soft reset finish
    retry=0;
    while(VS_RD_Reg(SPI_CLOCKF)!=0X9800)
    {
        VS_WR_Cmd(SPI_CLOCKF,0X9800);//set vs10xx clock
        if(retry++>100)break;
    }
    systick_delay_ms(20);
} 

/**
 * @brief  vs1053b hardware reset
* @return 1:reset fail; 0:reset success
*/   
uint8_t VS_HD_Reset(void)
{
    uint16_t retry=0;
    VS_RST = 0;
    systick_delay_ms(20);
    VS_XDCS = 1;
    VS_XCS = 1;
    VS_RST = 1;
    while(VS_DQ==0&&retry<200)//wait buf free
    {
        retry++;
        systick_delay_us(50);
    };
    systick_delay_ms(20);
    if(retry>=200)
    {
        return 1;
    }
    else 
    {
        return 0;
    }
}

/**
 * @brief  vs1053b sine test
*/ 
void VS_Sine_Test(void)
{
    VS_HD_Reset();
    VS_WR_Cmd(SPI_VOL,0x2020);  //set vol
    VS_WR_Cmd(SPI_MODE,0x0820); //enter test mode   
    while(VS_DQ==0);            //wait buf free
//printf("mode sin:%x\r\n",VS_RD_Reg(SPI_MODE));
//send sine test command:0x53 0xef 0x6e n 0x00 0x00 0x00 0x00
//n = 0x24, set VS10XX sine frequency
    VS_SPI_SpeedLow();//set spi speed low
    VS_XDCS=0;
    VS_SPI_ReadWriteByte(0x53);
    VS_SPI_ReadWriteByte(0xef);
    VS_SPI_ReadWriteByte(0x6e);
    VS_SPI_ReadWriteByte(0x24);
    VS_SPI_ReadWriteByte(0x00);
    VS_SPI_ReadWriteByte(0x00);
    VS_SPI_ReadWriteByte(0x00);
    VS_SPI_ReadWriteByte(0x00);
    systick_delay_ms(100);
    VS_XDCS=1; 

    //exit sine test
    VS_XDCS=0;
    VS_SPI_ReadWriteByte(0x45);
    VS_SPI_ReadWriteByte(0x78);
    VS_SPI_ReadWriteByte(0x69);
    VS_SPI_ReadWriteByte(0x74);
    VS_SPI_ReadWriteByte(0x00);
    VS_SPI_ReadWriteByte(0x00);
    VS_SPI_ReadWriteByte(0x00);
    VS_SPI_ReadWriteByte(0x00);
    systick_delay_ms(100);
    VS_XDCS=1;

    //send sine test command:0x53 0xef 0x6e n 0x00 0x00 0x00 0x00
    //n = 0x44, set VS10XX sine frequency
    VS_XDCS=0;
    VS_SPI_ReadWriteByte(0x53);
    VS_SPI_ReadWriteByte(0xef);
    VS_SPI_ReadWriteByte(0x6e);
    VS_SPI_ReadWriteByte(0x44);
    VS_SPI_ReadWriteByte(0x00);
    VS_SPI_ReadWriteByte(0x00);
    VS_SPI_ReadWriteByte(0x00);
    VS_SPI_ReadWriteByte(0x00);
    systick_delay_ms(100);
    VS_XDCS=1;
    
    //exit sine test
    VS_XDCS=0;
    VS_SPI_ReadWriteByte(0x45);
    VS_SPI_ReadWriteByte(0x78);
    VS_SPI_ReadWriteByte(0x69);
    VS_SPI_ReadWriteByte(0x74);
    VS_SPI_ReadWriteByte(0x00);
    VS_SPI_ReadWriteByte(0x00);
    VS_SPI_ReadWriteByte(0x00);
    VS_SPI_ReadWriteByte(0x00);
    systick_delay_ms(100);
    VS_XDCS=1;
}

/**
 * @brief  vs ram test
 * @return The result of ram test
 */
uint16_t VS_Ram_Test(void)
{ 
    VS_HD_Reset();
    VS_WR_Cmd(SPI_MODE,0x0820);// enter test mode
    while (VS_DQ == 0);
    VS_SPI_SpeedLow();//set spi speed low
    VS_XDCS=0;
    VS_SPI_ReadWriteByte(0x4d);
    VS_SPI_ReadWriteByte(0xea);
    VS_SPI_ReadWriteByte(0x6d);
    VS_SPI_ReadWriteByte(0x54);
    VS_SPI_ReadWriteByte(0x00);
    VS_SPI_ReadWriteByte(0x00);
    VS_SPI_ReadWriteByte(0x00);
    VS_SPI_ReadWriteByte(0x00);
    systick_delay_ms(100);  
    VS_XDCS=1;
    return VS_RD_Reg(SPI_HDAT0);//if VS1003 get value is 0x807F，it means OK ;VS1053 is 0X83FF.
}

/**
 * @brief  Sends cmd to vs1053b
* @param  address:register address; data:regsiter data
 */
void VS_WR_Cmd(uint8_t address,uint16_t data)
{  
    while(VS_DQ==0);//wait buf free
    VS_SPI_SpeedLow();//set spi speed low
    VS_XDCS=1;
    VS_XCS=0;
    VS_SPI_ReadWriteByte(VS_WRITE_COMMAND);//send write command
    VS_SPI_ReadWriteByte(address);  //send write address
    VS_SPI_ReadWriteByte(data>>8);  //send high byte
    VS_SPI_ReadWriteByte(data);     //send low byte
    VS_XCS=1;           
    VS_SPI_SpeedHigh();             //set spi speed high  
}

/**
 * @brief  Send data to vs1053b
* @param  data: data
 */
void VS_WR_Data(uint8_t data)
{
    VS_SPI_SpeedHigh();//for VS1003B,max speed 36.864/4Mhz，here is 9M 
    VS_XDCS=0;   
    VS_SPI_ReadWriteByte(data);
    VS_XDCS=1;      
}         

/**
* @brief  read vs1053b register data
* @param  address:register address;
* @return register data
 */
uint16_t VS_RD_Reg(uint8_t address)
{ 
    uint16_t temp=0;
    while(VS_DQ==0);//wait buf free
    VS_SPI_SpeedLow();//set speed low
    VS_XDCS=1;       
    VS_XCS=0;
    VS_SPI_ReadWriteByte(VS_READ_COMMAND);  //send vs1053b read command
    VS_SPI_ReadWriteByte(address);          //write read address
    temp=VS_SPI_ReadWriteByte(0xff);        //read high byte
    temp=temp<<8;
    temp+=VS_SPI_ReadWriteByte(0xff);       //read low byte
    VS_XCS=1;     
    VS_SPI_SpeedHigh();//set speed high 
    return temp; 
}


/**
* @brief  read vs1053b ram data
* @param  addr:ram address;
* @return ram data
 */
uint16_t VS_WRAM_Read(uint16_t addr) 
{ 
    uint16_t res;
    VS_WR_Cmd(SPI_WRAMADDR, addr); 
    res=VS_RD_Reg(SPI_WRAM);  
    return res;
} 

/**
* @brief  write vs1053b ram data
* @param  addr:ram address;  val:ram data
 */
void VS_WRAM_Write(uint16_t addr,uint16_t val) 
{
    VS_WR_Cmd(SPI_WRAMADDR,addr);   //write RAM address
    while(VS_DQ==0);                //wait buf free   
    VS_WR_Cmd(SPI_WRAM,val);        //writ ram
} 

/**
* @brief  set vs1053b speed
* @param  t:0,1:noram speed; 2:double speed; 3:triple speed; so on
 */
void VS_Set_Speed(uint8_t t)
{
    VS_WRAM_Write(0X1E04,t);    //set play speed
}
//FOR WAV HEAD0 :0X7761 HEAD1:0X7665    
//FOR MIDI HEAD0 :other info HEAD1:0X4D54
//FOR WMA HEAD0 :data speed HEAD1:0X574D
//FOR MP3 HEAD0 :data speed HEAD1:ID
const uint16_t bitrate[2][16]=
{ 
    {0,8,16,24,32,40,48,56,64,80,96,112,128,144,160,0}, 
    {0,32,40,48,56,64,80,96,112,128,160,192,224,256,320,0}
};

/**
 * @brief  get vs head info
 * @return The value bit rate /Kbps
 */
uint16_t VS_Get_HeadInfo(void)
{
    unsigned int HEAD0;
    unsigned int HEAD1;  
    HEAD0=VS_RD_Reg(SPI_HDAT0); 
    HEAD1=VS_RD_Reg(SPI_HDAT1);
    //printf("(H0,H1):%x,%x\n",HEAD0,HEAD1);
    switch(HEAD1)
    {        
        case 0x7665://WAV format
        case 0X4D54://MIDI format
        case 0X4154://AAC_ADTS
        case 0X4144://AAC_ADIF
        case 0X4D34://AAC_MP4/M4A
        case 0X4F67://OGG
        case 0X574D://WMA format
        case 0X664C://FLAC format
        {
            HEAD1=HEAD0*2/25;
            if((HEAD1%10)>5)return HEAD1/10+1;
            else return HEAD1/10;
        }
        default://MP3 Format
        {
            HEAD1>>=3;
            HEAD1=HEAD1&0x03; 
            if(HEAD1==3)HEAD1=1;
            else HEAD1=0;
            return bitrate[HEAD1][HEAD0>>12];
        }
    }  
}

/**
 * @brief  get vs average byte rate
 * @return average byte speed
 */
uint32_t VS_Get_ByteRate(void)
{
    return VS_WRAM_Read(0X1E05);
}

/**
 * @brief  get vs need fill byte count
 * @return need fill byte count
 */
uint16_t VS_Get_EndFillByte(void)
{
    return VS_WRAM_Read(0X1E06);
}

/**
* @brief  send music data to vs1053b   send 32 byte
* @param  buf:music data
* @return 1:send fail; 0:send success
 */
uint8_t VS_Send_MusicData(uint8_t* buf)
{
    uint8_t n;

    if(VS_DQ!=0)
    {
        VS_XDCS=0;  
        for(n=0;n<AUDIO_DATA_LENGTH;n++)
        {
            VS_SPI_ReadWriteByte(buf[n]);
        }
        VS_XDCS=1;  
    }
    else
    {
        return 1;
    }
    return 0;
}
/**
 * @brief  restart play
 */
void VS_Restart_Play(void)
{
    uint16_t temp;
    uint16_t i;
    uint8_t n;
    uint8_t vsbuf[32];
    for(n=0;n<32;n++)vsbuf[n]=0;
    temp=VS_RD_Reg(SPI_MODE);   //get spi mode
    temp|=1<<3;                 //set SM_CANCEL bit
    temp|=1<<2;                 //set SM_LAYER12 bit ,support MP1,MP2
    VS_WR_Cmd(SPI_MODE,temp);   //cancel present decode command
    for(i=0;i<2048;i++)
    {
        if(VS_Send_MusicData(vsbuf)==0)//check after send 32 bytes every time
        {
            i+=32;                  //send 32 bytes
        temp=VS_RD_Reg(SPI_MODE);   //get spi mode
            if((temp&(1<<3))==0)break;  //succeed
        }
    }
    if(i<2048)//SM_CANCEL normal
    {
        temp=VS_Get_EndFillByte()&0xff;//get fill byte
        for(n=0;n<32;n++)vsbuf[n]=temp;//fill byte
        for(i=0;i<2052;)
        {
            if(VS_Send_MusicData(vsbuf)==0)i+=32;//fill  
        }
    }else VS_Soft_Reset();  //SM_CANCEL fail, need soft reset  
    temp=VS_RD_Reg(SPI_HDAT0); 
    temp+=VS_RD_Reg(SPI_HDAT1);
    if(temp)                //if soft reset fail, hardware reset
    {
        VS_HD_Reset();      //hardware reset
        VS_Soft_Reset();    //soft reset
    } 
}
/**
 * @brief  reset vs decode time
 */                        
void VS_Reset_DecodeTime(void)
{
    VS_WR_Cmd(SPI_DECODE_TIME,0x0000);
    VS_WR_Cmd(SPI_DECODE_TIME,0x0000);//operate twice
}

/**
* @brief  get vs decode time
* @return decode time
 */
uint16_t VS_Get_DecodeTime(void)
{
    uint16_t dt=0;
    dt=VS_RD_Reg(SPI_DECODE_TIME);
    return dt;
}

/**
* @brief  vs1053b load patch
* @param  patch:patch addr; len:patch length
 */
void VS_Load_Patch(uint16_t *patch,uint16_t len) 
{
    uint16_t i; 
    uint16_t addr, n, val;
    for(i=0;i<len;) 
    { 
        addr = patch[i++]; 
        n    = patch[i++]; 
        if(n & 0x8000U) //RLE run, replicate n samples 
        { 
            n  &= 0x7FFF; 
            val = patch[i++]; 
            while(n--)VS_WR_Cmd(addr, val);  
        }else //copy run, copy n sample 
        { 
            while(n--) 
            { 
                val = patch[i++]; 
                VS_WR_Cmd(addr, val); 
            } 
        } 
    }
}

/**
* @brief  vs1053b set vol
* @param  volx: vol value
 */
void VS_Set_Vol(uint8_t volx)
{
    uint16_t volt=0;
    volt=254-volx;
    volt<<=8;
    volt+=254-volx;
    VS_WR_Cmd(SPI_VOL,volt);//set vol
}

/**
* @brief  vs1053b set vol
* @param  bfreq: low frequency upper limit frequency    2~15(unit:10Hz)
* @param  bass: low frequency gain                      0~15(unit:1dB)
* @param  tfreq: high frequency upper limit frequency   1~15(unit:Khz)
* @param  treble: high frequency gain                   0~15(unit:1.5dB,小于9的时候为负数)
 */
void VS_Set_Bass(uint8_t bfreq,uint8_t bass,uint8_t tfreq,uint8_t treble)
{
    uint16_t bass_set=0;
    signed char temp=0;
    if(treble==0)temp=0;
    else if(treble>8)temp=treble-8;
    else temp=treble-9;  
    bass_set=temp&0X0F;         //treble set
    bass_set<<=4;
    bass_set+=tfreq&0xf;        //low limit of treble
    bass_set<<=4;
    bass_set+=bass&0xf;         //bass set
    bass_set<<=4;
    bass_set+=bfreq&0xf;        //upper limit of the bass
    VS_WR_Cmd(SPI_BASS,bass_set);   //BASS 
}

/**
* @brief  set sound effect
* @param  eft: 0:close, 1:min, 2:medium, 3:max
 */
void VS_Set_Effect(uint8_t eft)
{
    uint16_t temp;
    temp=VS_RD_Reg(SPI_MODE);
    if(eft&0X01)temp|=1<<4;
    else temp&=~(1<<5);
    if(eft&0X02)temp|=1<<7;
    else temp&=~(1<<7);
    VS_WR_Cmd(SPI_MODE,temp);
}
  
/**
* @brief  set speaker
* @param  sw:0,close;1,open
 */
void VS_Set_SPK(uint8_t sw)
{
    if(sw)
    {
        GPIO_SetBits(SPK_CTL_PORT, SPK_CTL_PIN);
    }
    else
    {
        GPIO_ResetBits(SPK_CTL_PORT, SPK_CTL_PIN);
    }
} 

/**
* @brief  vs1053b set vol、effect
* @param  patch:patch addr; len:patch length
 */
void VS_Set_All(void)
{
    VS_Set_Vol(vsset.mvol);         //set the volume
    VS_Set_Bass(vsset.bflimit,vsset.bass,vsset.tflimit,vsset.treble);  
    VS_Set_Effect(vsset.effect);    //set sound effect
}


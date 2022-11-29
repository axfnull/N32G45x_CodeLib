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
 * @file vs10xx.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef __VS10XX_H__
#define __VS10XX_H__

#include "stdint.h"

#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 

#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011C0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x4001200C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011C08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40012008 


#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n) 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n) 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  


#define VS_DQ       PAin(4)     //DREQ PA4
#define VS_RST      PBout(0)    //RST  PB0
#define VS_XCS      PCout(2)    //XCS  PC2
#define VS_XDCS     PAout(3)    //XDCS PA3


#define VS_DREQ_PIN         GPIO_PIN_4
#define VS_RST_PIN          GPIO_PIN_0
#define VS_XCS_PIN          GPIO_PIN_2
#define VS_XDCS_PIN         GPIO_PIN_3

#define VS_DREQ_PORT        GPIOA
#define VS_RST_PORT         GPIOB
#define VS_XCS_PORT         GPIOC
#define VS_XDCS_PORT        GPIOA

#define SPK_CTL_PIN         GPIO_PIN_12
#define SPK_CTL_PORT        GPIOC

typedef struct 
{                             
    uint8_t mvol;           //main vol,range:0~254
    uint8_t bflimit;        //bflimit,range:2~15(unit:10Hz)
    uint8_t bass;           //bass,range:0~15.0,close.(unit:1dB)
    uint8_t tflimit;        //tflimit,range:1~15(unit:Khz)
    uint8_t treble;         //treble,range:0~15(unit:1.5dB)
    uint8_t effect;         //effect.0,close; 1,min, 2,medium; 3,max
    uint8_t speakersw;      //speaker 0,close;1,open 
    uint8_t saveflag;       //save flag, 0X0A,save   
}_vs10xx_obj;

extern _vs10xx_obj vsset;   

#define VS_WRITE_COMMAND    0x02
#define VS_READ_COMMAND     0x03

//VS10XX register
#define SPI_MODE            0x00   
#define SPI_STATUS          0x01   
#define SPI_BASS            0x02   
#define SPI_CLOCKF          0x03   
#define SPI_DECODE_TIME     0x04   
#define SPI_AUDATA          0x05   
#define SPI_WRAM            0x06   
#define SPI_WRAMADDR        0x07   
#define SPI_HDAT0           0x08   
#define SPI_HDAT1           0x09 
  
#define SPI_AIADDR          0x0a   
#define SPI_VOL             0x0b   
#define SPI_AICTRL0         0x0c   
#define SPI_AICTRL1         0x0d   
#define SPI_AICTRL2         0x0e   
#define SPI_AICTRL3         0x0f   
#define SM_DIFF             0x01   
#define SM_JUMP             0x02   
#define SM_RESET            0x04   
#define SM_OUTOFWAV         0x08   
#define SM_PDOWN            0x10   
#define SM_TESTS            0x20   
#define SM_STREAM           0x40   
#define SM_PLUSV            0x80   
#define SM_DACT             0x100   
#define SM_SDIORD           0x200   
#define SM_SDISHARE         0x400   
#define SM_SDINEW           0x800   
#define SM_ADPCM            0x1000   
#define SM_ADPCM_HP         0x2000       

#define I2S_CONFIG          0XC040
#define GPIO_DDR            0XC017
#define GPIO_IDATA          0XC018
#define GPIO_ODATA          0XC019



uint16_t  VS_RD_Reg(uint8_t address);           //read register
uint16_t  VS_WRAM_Read(uint16_t addr);          //read RAM
void VS_WRAM_Write(uint16_t addr,uint16_t val); //write RAM
void VS_WR_Data(uint8_t data);                  //write data
void VS_WR_Cmd(uint8_t address,uint16_t data);  //write commadn
uint8_t   VS_HD_Reset(void);                    //hardware reset
void VS_Soft_Reset(void);                       //software reset
uint16_t VS_Ram_Test(void);                     //RAM test    
void VS_Sine_Test(void);                        //sine test
                                                     
uint8_t      VS_SPI_ReadWriteByte(uint8_t data);
void VS_SPI_SpeedLow(void);
void VS_SPI_SpeedHigh(void);
void VS_Init(void);                         //VS10XX init    
void VS_Set_Speed(uint8_t t);               //set play speed
uint16_t  VS_Get_HeadInfo(void);            //get head info
uint32_t VS_Get_ByteRate(void);             //get byte rate
uint16_t VS_Get_EndFillByte(void);      
uint8_t      VS_Send_MusicData(uint8_t* buf);   //send music data to vs
void VS_Restart_Play(void);                     //restart play    
void VS_Reset_DecodeTime(void);                 //reset decode time
uint16_t  VS_Get_DecodeTime(void);              //get decode time

void VS_Load_Patch(uint16_t *patch,uint16_t len);   //load patch
uint8_t      VS_Get_Spec(uint16_t *p);
void VS_Set_Bands(uint16_t *buf,uint8_t bands); 
void VS_Set_Vol(uint8_t volx);              //set volumn
void VS_Set_Bass(uint8_t bfreq,uint8_t bass,uint8_t tfreq,uint8_t treble);//set bass
void VS_Set_Effect(uint8_t eft);            //set sound effect
void VS_Set_SPK(uint8_t sw);
void VS_Set_All(void);

void vs10xx_read_para(_vs10xx_obj * vs10xxdev);
void vs10xx_save_para(_vs10xx_obj * vs10xxdev);

#endif

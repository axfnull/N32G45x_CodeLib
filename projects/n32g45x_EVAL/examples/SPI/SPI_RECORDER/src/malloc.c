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
 * @file malloc.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include "malloc.h"     


uint8_t membase[MEM_MAX_SIZE];                  //SRAM memory pool 

uint16_t memmapbase[MEM_ALLOC_TABLE_SIZE];      //SRAM memory pool MAP

const uint32_t memtblsize=MEM_ALLOC_TABLE_SIZE;
const uint32_t memblksize=MEM_BLOCK_SIZE;
const uint32_t memsize=MEM_MAX_SIZE;

struct _m_mallco_dev mallco_dev=
{
    mem_init,
    mem_perused,
    membase,
    memmapbase,
    0,
};

/**
 * @brief  copy data
 * @param des target address.
 * @param src source address.
 * @param n   data length.
 */
void mymemcpy(void *des,void *src,uint32_t n)  
{
    uint8_t *xdes=des;
    uint8_t *xsrc=src; 
    while(n--)*xdes++=*xsrc++;  
}  

/**
 * @brief  set data
 * @param s target address.
 * @param c data.
 * @param count   data length.
 */
void mymemset(void *s,uint8_t c,uint32_t count)  
{
    uint8_t *xs = s;  
    while(count--)*xs++=c;  
}   

/**
 * @brief  memory pool init
 */ 
void mem_init(void)  
{
    mymemset(mallco_dev.memmap, 0,memtblsize*2);
    mymemset(mallco_dev.membase, 0,memsize);
    mallco_dev.memrdy=1;
}  

/**
 * @brief  caculate memory using percent
*  @return using percent 
 */ 
uint8_t mem_perused(void)  
{
    uint32_t used=0;  
    uint32_t i;  
    for(i=0;i<memtblsize;i++)  
    {
        if(mallco_dev.memmap[i])used++; 
    }
    return (used*100)/(memtblsize);  
}  

/**
 * @brief  malloc memory (inside)
 * @param  size memory size
 * @return memeory offset address 
 */ 
uint32_t mem_malloc(uint32_t size)  
{  
    signed long offset=0;  
    uint16_t nmemb;
    uint16_t cmemb=0;
    uint32_t i;  
    if(!mallco_dev.memrdy)mallco_dev.init();    
    if(size==0)return 0XFFFFFFFF;               
    nmemb=size/memblksize;                      
    if(size%memblksize)nmemb++;
    for(offset=memtblsize-1;offset>=0;offset--)
    {
        if(!mallco_dev.memmap[offset])cmemb++;
        else cmemb=0;
        if(cmemb==nmemb)
        {
            for(i=0;i<nmemb;i++)                
            {
                mallco_dev.memmap[offset+i]=nmemb;  
            }  
            return (offset*memblksize);
        }
    }  
    return 0XFFFFFFFF;//malloc memory fail
}  

/**
 * @brief  free the memory (inside)
 * @param  offset offset address
 * @return free result
 */
uint8_t mem_free(uint32_t offset)  
{  
    int i;  
    if(!mallco_dev.memrdy)
    {
        mallco_dev.init();    
        return 1;
    }  
    if(offset<memsize)
    {  
        int index=offset/memblksize;
        int nmemb=mallco_dev.memmap[index];
        for(i=0;i<nmemb;i++)
        {  
            mallco_dev.memmap[index+i]=0;  
        }
        return 0;  
    }else return 2;//offset super area
}

/**
 * @brief  free the memory (outside)
 * @param  ptr offset address
 */
void myfree(void *ptr)  
{  
    uint32_t offset;  
    if(ptr==NULL)return;  
    offset=(uint32_t)ptr-(uint32_t)mallco_dev.membase;  
    mem_free(offset);
}

/**
 * @brief  malloc memory (outside)
 * @param  size memory size
 * @return memeory first address 
 */ 
void *mymalloc(uint32_t size)  
{  
    uint32_t offset;                                          
    offset=mem_malloc(size);                       
    if(offset==0XFFFFFFFF)return NULL;  
    else return (void*)((uint32_t)mallco_dev.membase+offset);  
}  

/**
 * @brief  redistribut malloc memory
 * @param  ptr old memory first address
 * @return new memory first address 
 */ 
void *myrealloc(void *ptr,uint32_t size)  
{  
    uint32_t offset;  
    offset=mem_malloc(size);  
    if(offset==0XFFFFFFFF)return NULL;     
    else
    {                      
        mymemcpy((void*)((uint32_t)mallco_dev.membase+offset),ptr,size);    
        myfree(ptr);      
        return (void*)((uint32_t)mallco_dev.membase+offset); 
    }  
}

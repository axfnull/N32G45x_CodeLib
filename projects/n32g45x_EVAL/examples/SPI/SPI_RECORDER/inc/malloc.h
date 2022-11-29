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
 * @file malloc.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef __MALLOC_H
#define __MALLOC_H
#include "n32g45x.h"
 
#ifndef NULL
#define NULL 0
#endif


#define MEM_BLOCK_SIZE          32
#define MEM_MAX_SIZE            42*1024
#define MEM_ALLOC_TABLE_SIZE    MEM_MAX_SIZE/MEM_BLOCK_SIZE
 

struct _m_mallco_dev
{
    void (*init)(void);                 //memory init
    uint8_t (*perused)(void);           //memory usage
    uint8_t  *membase;                  //memory pool
    uint16_t *memmap;                   //memory management state table
    uint8_t  memrdy;                    //memory management state
};
extern struct _m_mallco_dev mallco_dev;

void mymemset(void *s,uint8_t c,u32 count); //memory set
void mymemcpy(void *des,void *src,u32 n);   //memory copy
void mem_init(void);                        //memory init
u32 mem_malloc(u32 size);                   //memory allocation(internal call)
uint8_t mem_free(u32 offset);               //memory free(internal call)
uint8_t mem_perused(void);                  //memory usage(internal call)

//user call
void myfree(void *ptr);                 //memory free(outside call)
void *mymalloc(u32 size);               //memory allocation(outside call)
void *myrealloc(void *ptr,u32 size);    //memory reallocation(outside call)
#endif


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
 * @file application.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include <rtthread.h>
#include "rtdef.h"
#include "log.h"
#include "drv_gpio.h"
#include "pin.h"
#include "drv_spi.h"
#include "spi_flash.h"

#ifdef RT_USING_DFS
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif

#ifdef RT_USING_RTGUI
#include <rtgui/rtgui.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/driver.h>
#include <rtgui/calibration.h>
#endif

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t InitStack[2048], led0_stack[ 512 ], led1_stack[ 512 ];
static struct rt_thread InitHandle;
static struct rt_thread led0_thread;
static struct rt_thread led1_thread;

struct rt_event system_event;

#define LED0_PIN        GET_PIN(B,  10)
#define LED1_PIN        GET_PIN(B,  15)

#define KEY1_PIN        GET_PIN(C,  6)
#define KEY2_PIN        GET_PIN(C,  7)

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

TestStatus Buffercmp8bit(uint8_t* pBuffer, uint8_t* pBuffer1, uint16_t BufferLength);
TestStatus Buffercmp16bit(uint16_t* pBuffer, uint16_t* pBuffer1, uint16_t BufferLength);
TestStatus Buffercmp32bit(uint32_t* pBuffer, uint32_t* pBuffer1, uint16_t BufferLength);
TestStatus TransferStatus1 = FAILED, TransferStatus2 = PASSED;;

/* The interrupt callback function */
void key_press(void *args)
{
    rt_kprintf("KEY2_PIN IS PRESS!\n");
}

/**
 * @brief  led0 thread entry
 */
static void led0_thread_entry(void* parameter)
{
    while(1)
    {
        rt_thread_delay(RT_TICK_PER_SECOND/2);  //delay 500ms
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_delay(RT_TICK_PER_SECOND/2);  //delay 500ms
        rt_pin_write(LED0_PIN, PIN_LOW);
    }
}

/**
 * @brief  led1 thread entry
 */
static void led1_thread_entry(void* parameter)
{
    while(1)
    {
        if(rt_pin_read(KEY1_PIN))
        {
            rt_pin_write(LED1_PIN, PIN_HIGH);
        }
        else
        {
            rt_pin_write(LED1_PIN, PIN_LOW);
        }
        
        rt_thread_delay(10);    //delay 100ms

    }
}

#ifdef RT_USING_RTGUI
rt_bool_t cali_setup(void)
{
    rt_kprintf("cali setup entered\n");
    return RT_FALSE;
}

void cali_store(struct calibration_data *data)
{
    rt_kprintf("cali finished (%d, %d), (%d, %d)\n",
               data->min_x,
               data->max_x,
               data->min_y,
               data->max_y);
}
#endif /* RT_USING_RTGUI */

#define FLASH_WriteAddress  0x700000

static void InitTask(void* parameter)
{    
    rt_err_t result;
    rt_device_t storage_dev;   
    uint32_t device_id = 0;
    rt_uint32_t index = FLASH_WriteAddress;    
    uint8_t data1[300] = {0};
    uint8_t data2[300] = {0};
    uint16_t Index = 0x0;
    
    rt_hw_pin_init();
    rt_hw_spi_init();                           //register spi bus  

    rt_flash_register(0);                       //register flash device
    storage_dev = rt_device_find("Flash");    
    
    rt_pin_mode(KEY2_PIN, PIN_MODE_INPUT_PULLDOWN);    
    rt_pin_attach_irq(KEY2_PIN, PIN_IRQ_MODE_RISING, key_press, RT_NULL);
    rt_pin_irq_enable(KEY2_PIN, PIN_IRQ_ENABLE);
    
    if(RT_EOK == rt_device_open(storage_dev, 0))//open flash device
    {
        device_id = rt_device_control(storage_dev, RT_CMD_W25_READ_ID, RT_NULL);
        if(device_id == sFLASH_W25Q128_ID)
        {
            rt_device_control(storage_dev, RT_CMD_W25_ERASE_SECTOR, &index);
            rt_device_write(storage_dev, FLASH_WriteAddress, data1, 300);
            rt_device_read(storage_dev, FLASH_WriteAddress, data2, 300); 
            
            TransferStatus1 = Buffercmp8bit(data1, data2, 300);     
            
            rt_device_control(storage_dev, RT_CMD_W25_ERASE_SECTOR, &index);
            rt_device_read(storage_dev, FLASH_WriteAddress, data2, 300);
            
            /* Check the correctness of erasing operation dada */
            for (Index = 0; Index < 300; Index++)
            {
                if (data2[Index] != 0xFF)
                {
                    TransferStatus2 = FAILED;
                }
            }
            
            if (TransferStatus1 == PASSED && TransferStatus2 == PASSED)
            {
                rt_kprintf("the write and read data are the same,SPI FLASH test pass\r\n");
            }
            else
            {
                rt_kprintf("the write and read data are different,SPI FLASH test fail\r\n");
            }
        }        
    }
    
    /* init led0 thread */
    result = rt_thread_init(&led0_thread, "led0", led0_thread_entry, RT_NULL, (rt_uint8_t*)&led0_stack[0], sizeof(led0_stack), 3, 5);
    if (result == RT_EOK)
    {
        rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
        rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
        rt_thread_startup(&led0_thread);
    }
    /* init led1 thread */
    result = rt_thread_init(&led1_thread, "led1", led1_thread_entry, RT_NULL, (rt_uint8_t*)&led1_stack[0], sizeof(led1_stack), 3, 5);
    if (result == RT_EOK)
    {
        rt_pin_mode(KEY1_PIN, PIN_MODE_INPUT_PULLDOWN);
        rt_thread_startup(&led1_thread);
    }
    while(1)
    {
        rt_thread_delay(50);        
    }
}

/**
 * @brief  init application
 */
void rt_application_init(void)
{
    rt_err_t result;

    result = rt_thread_init(&InitHandle, "Init", InitTask, RT_NULL, (rt_uint8_t*)&InitStack[0], sizeof(InitStack), 2, 5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&InitHandle);
    }
   
}


/**
 * @brief  Compares two buffers.
 * @param  pBuffer, pBuffer1: buffers to be compared.
 * @param BufferLength buffer's length
 * @return PASSED: pBuffer identical to pBuffer1
 *         FAILED: pBuffer differs from pBuffer1
 */
TestStatus Buffercmp8bit(uint8_t* pBuffer, uint8_t* pBuffer1, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer != *pBuffer1)
        {
            return FAILED;
        }

        pBuffer++;
        pBuffer1++;
    }

    return PASSED;
}

/**
 * @brief  Compares two buffers.
 * @param  pBuffer, pBuffer1: buffers to be compared.
 * @param BufferLength buffer's length
 * @return PASSED: pBuffer identical to pBuffer1
 *         FAILED: pBuffer differs from pBuffer1
 */
TestStatus Buffercmp16bit(uint16_t* pBuffer, uint16_t* pBuffer1, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer != *pBuffer1)
        {
            return FAILED;
        }

        pBuffer++;
        pBuffer1++;
    }

    return PASSED;
}

/**
 * @brief  Compares two buffers.
 * @param  pBuffer, pBuffer1: buffers to be compared.
 * @param BufferLength buffer's length
 * @return PASSED: pBuffer identical to pBuffer1
 *         FAILED: pBuffer differs from pBuffer1
 */
TestStatus Buffercmp32bit(uint32_t* pBuffer, uint32_t* pBuffer1, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer != *pBuffer1)
        {
            return FAILED;
        }

        pBuffer++;
        pBuffer1++;
    }

    return PASSED;
}

/*@}*/

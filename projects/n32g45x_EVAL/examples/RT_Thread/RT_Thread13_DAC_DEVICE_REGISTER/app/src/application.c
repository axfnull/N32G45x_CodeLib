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
#include "drv_adc.h"
#include "drv_dac.h"

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
static rt_uint8_t InitStack[1024], led0_stack[ 512 ], led1_stack[ 512 ];
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
volatile TestStatus TransferStatus1 = FAILED;

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

static void InitTask(void* parameter)
{    
    rt_err_t result;
    rt_adc_device_t adc_dev;    
    rt_dac_device_t dac_dev;
    uint16_t adc_converted_value = 0;
    uint16_t dac_set_value = 0;
    
    rt_hw_pin_init();  
    rt_hw_adc_init();                   //register adc
    rt_hw_dac_init();                   //register dac
    
    rt_pin_mode(KEY2_PIN, PIN_MODE_INPUT_PULLDOWN);    
    rt_pin_attach_irq(KEY2_PIN, PIN_IRQ_MODE_RISING, key_press, RT_NULL);
    rt_pin_irq_enable(KEY2_PIN, PIN_IRQ_ENABLE);
    
    adc_dev = (rt_adc_device_t)rt_device_find("adc1");
    rt_adc_enable(adc_dev, ADC1_Channel_06_PC0);
    
    dac_dev = (rt_dac_device_t)rt_device_find("dac1");
    rt_dac_enable(dac_dev, RT_NULL);
    
    if(dac_dev)
    {
        rt_dac_write(dac_dev, DAC_CHANNEL_1, 2048);
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
        rt_thread_delay(10);
        dac_set_value += 64;
        rt_dac_write(dac_dev, DAC_CHANNEL_1, dac_set_value);
        adc_converted_value = rt_adc_read(adc_dev, ADC1_Channel_06_PC0);
        rt_kprintf("the PC0 voltage value is %d\r\n", adc_converted_value);
        if(dac_set_value > 4096)
        {
            dac_set_value = 0;
        }
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

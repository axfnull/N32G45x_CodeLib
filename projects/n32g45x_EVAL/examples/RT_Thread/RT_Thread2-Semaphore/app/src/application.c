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
#include "led.h"
#include "key.h"

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
static rt_uint8_t InitStack[512], led0_stack[ 512 ], led1_stack[ 512 ];
static rt_uint8_t key_stack[512];
static struct rt_thread InitHandle;
static struct rt_thread led0_thread;
static struct rt_thread led1_thread;
static struct rt_thread key_thread;

struct rt_semaphore key_sem;

/**
 * @brief  led0 thread entry
 */
static void led0_thread_entry(void* parameter)
{
    rt_uint8_t led_state = 0;
    while(1)
    {
        /* wait get sem forever*/
        rt_sem_take(&key_sem, RT_WAITING_FOREVER);
        led_state ^=1;
        if (led_state!=0)
        {
            rt_hw_led_on(0);
            rt_kprintf(" get semaphore ok, led on \r\n");
        }
        else
        {
            rt_hw_led_off(0);
            rt_kprintf(" get semaphore ok, led off \r\n");
        }
    }
}

/**
 * @brief  led1 thread entry
 */
static void led1_thread_entry(void* parameter)
{
    while(1)
    {
        rt_thread_delay(25);    //delay 250ms
        rt_hw_led_on(1);
        rt_thread_delay(25);    //delay 250ms
        rt_hw_led_off(1);
    }
}

/**
 * @brief  led1 thread entry
 */
void key_thread_entry(void* parameter)
{
    rt_err_t result;
    rt_uint8_t key;

    KEY_Init();

    /* init keysem */
    result = rt_sem_init(&key_sem, "keysem", 0, RT_IPC_FLAG_FIFO);
    if (result != RT_EOK)
    {
        rt_kprintf("init keysem failed.\n");
    }

    while(1)
    {
        key = KEY_Scan(0);
        switch(key)
        {
            case KEY0_PRES:
                rt_sem_release(&key_sem);
                break;
            case KEY1_PRES:
                break;
            case KEY2_PRES:
                break;

            default:
                break;
            
        }
        rt_thread_delay(5);
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

    rt_hw_led0_init();
    rt_hw_led1_init();

    /* init led0 thread */
    result = rt_thread_init(&led0_thread, "led0", led0_thread_entry, RT_NULL, (rt_uint8_t*)&led0_stack[0], sizeof(led0_stack), 16, 5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&led0_thread);
    }
    /* init led1 thread */
    result = rt_thread_init(&led1_thread, "led1", led1_thread_entry, RT_NULL, (rt_uint8_t*)&led1_stack[0], sizeof(led1_stack), 17, 5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&led1_thread);
    }
    /* init key thread */
    result = rt_thread_init(&key_thread, "key", key_thread_entry, RT_NULL, (rt_uint8_t*)&key_stack[0], sizeof(key_stack), 15, 5);
    {
        rt_thread_startup(&key_thread);
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

/*@}*/

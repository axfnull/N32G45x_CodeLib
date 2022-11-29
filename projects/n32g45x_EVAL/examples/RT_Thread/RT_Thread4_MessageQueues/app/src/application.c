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

#include "rtthread.h"
#include "n32g45x.h"
#include "rtdef.h"
#include "key.h"
#include "led.h"

ALIGN(RT_ALIGN_SIZE)

static rt_uint8_t InitStack[512], led0_stack[ 512 ];
static rt_uint8_t key_stack[512];
static struct rt_thread InitHandle;
static struct rt_thread led0_thread;
static struct rt_thread key_thread;

struct rt_messagequeue mq;
static char msg_pool[2048];

/**
 * @brief  led0 thread entry
 */
static void led0_thread_entry(void* parameter)
{
    rt_uint8_t led_state = 0;
    char buf[128];

    rt_hw_led0_init();
    rt_hw_led1_init();
    rt_hw_led_off(0);
    rt_hw_led_off(1);
    rt_memset(&buf[0], 0, sizeof(buf));
    while(1)
    {
        /* recv message queue */
        if (rt_mq_recv(&mq, &buf[0], sizeof(buf), RT_WAITING_FOREVER) == RT_EOK)
        {
            rt_kprintf("led: recv msg from msg queue, the content:%s\n", buf);
        
            if (buf[4]=='1') led_state =0x01;
            if (buf[4]=='2') led_state =0x02;
            if (buf[4]=='3') led_state =0x03;
        }
        if (led_state==1)
        {
            rt_hw_led_on(0);
        }
        else
        {
            rt_hw_led_off(0);
        }
        if (led_state==2)
        {
            rt_hw_led_on(1);
        }
        else
        {
            rt_hw_led_off(1);
        }
    }
}

static char key_info[]= "key x has been pressed!";
/**
 * @brief  led1 thread entry
 */
void key_thread_entry(void* parameter)
{
    rt_uint8_t key;

    KEY_Init();

    while(1)
    {
        key = KEY_Scan(0);
        switch(key)
        {
            case KEY0_PRES:
                key_info[4]='1';
                break;
            case KEY1_PRES:
                key_info[4]='2';
                break;
            case KEY2_PRES:
                key_info[4]='3';
                break;

            default:
                break;
        }
        if(key_info[4]=='1' || key_info[4]=='2' || key_info[4]=='3')
        {
            /* send message queue */
            rt_mq_send(&mq, &key_info[0], sizeof(key_info));
            key_info[4]=0;
        }
    }
}

/**
 * @brief  Init thread entry
 */
static void InitTask(void* parameter)
{
    rt_err_t result;

    /* init message queue */
    result =  rt_mq_init(&mq, "mqt",
                         &msg_pool[0],
                         128- sizeof(void*), 
                         sizeof(msg_pool), 
                         RT_IPC_FLAG_FIFO);
    if (result != RT_EOK)
    {
        rt_kprintf("init message queue failed.\n");
    }

   /* init led0 thread */
    result = rt_thread_init(&led0_thread, "led0", led0_thread_entry, RT_NULL, (rt_uint8_t*)&led0_stack[0], sizeof(led0_stack), 14, 5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&led0_thread);
    }

    /* init key thread */
    result = rt_thread_init(&key_thread, "key", key_thread_entry, RT_NULL, (rt_uint8_t*)&key_stack[0], sizeof(key_stack), 15, 5);
    {
        rt_thread_startup(&key_thread);
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

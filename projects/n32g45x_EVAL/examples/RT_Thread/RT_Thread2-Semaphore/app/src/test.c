/**********************************************************************************************************
*
*	模块名称 : 功能演示
*	文件名称 : test.c
*	版    本 : V1.0
*	说    明 :
*	修改记录 :
*		版本号  日期        作者                        说明
*
*		v1.0    2013-4-20   jiezhi320(UP MCU 工作室)    演示信号量的实际使用
*
*	Copyright (C), 2012-2013,
*   淘宝店：   http://shop73275611.taobao.com
*   QQ交流群： 258043068
*
**********************************************************************************************************/
#include <rtthread.h>
#include <n32g45x.h>
#include "test.h"
#include "key.h"
#include "led.h"

/*  变量分配4字节对齐 */
ALIGN(RT_ALIGN_SIZE)

/*  静态线程的 线程堆栈*/
static rt_uint8_t led_stack[512];
static rt_uint8_t key_stack[512];
/* 静态线程的 线程控制块 */
static struct rt_thread led_thread;
static struct rt_thread key_thread;

/* 信号量控制块 */
struct rt_semaphore key_sem;

rt_err_t demo_thread_creat(void)
{
    rt_err_t result;

    /* 初始化静态信号量，初始值是0 */
    result = rt_sem_init(&key_sem, "keysem", 0, RT_IPC_FLAG_FIFO);
    if (result != RT_EOK)
    {
        rt_kprintf("init keysem failed.\n");
        return -1;
    }

    /* 创建key线程 ： 优先级 15 ，时间片 5个系统滴答 */
    result = rt_thread_init(&key_thread,
                            "key",
                            key_thread_entry, RT_NULL,
                            (rt_uint8_t*)&key_stack[0], sizeof(key_stack), 15, 5);

    if (result == RT_EOK)
    {
        rt_thread_startup(&key_thread);
    }
    return 0;
}




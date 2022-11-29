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
 * @file completion.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "completion.h"

#define RT_COMPLETED    1
#define RT_UNCOMPLETED  0

void rt_completion_init(struct rt_completion *completion)
{
    rt_base_t level;
    RT_ASSERT(completion != RT_NULL);

    level = rt_hw_interrupt_disable();
    completion->flag = RT_UNCOMPLETED;
    rt_list_init(&completion->suspended_list);
    rt_hw_interrupt_enable(level);
}
RTM_EXPORT(rt_completion_init);

rt_err_t rt_completion_wait(struct rt_completion *completion,
                            rt_int32_t            timeout)
{
    rt_err_t result;
    rt_base_t level;
    rt_thread_t thread;
    RT_ASSERT(completion != RT_NULL);

    result = RT_EOK;
    thread = rt_thread_self();

    level = rt_hw_interrupt_disable();
    if (completion->flag != RT_COMPLETED)
    {
        /* only one thread can suspend on complete */
        RT_ASSERT(rt_list_isempty(&(completion->suspended_list)));

        if (timeout == 0)
        {
            result = -RT_ETIMEOUT;
            goto __exit;
        }
        else
        {
            /* reset thread error number */
            thread->error = RT_EOK;

            /* suspend thread */
            rt_thread_suspend(thread);
            /* add to suspended list */
            rt_list_insert_before(&(completion->suspended_list),
                                  &(thread->tlist));

            /* current context checking */
            RT_DEBUG_NOT_IN_INTERRUPT;

            /* start timer */
            if (timeout > 0)
            {
                /* reset the timeout of thread timer and start it */
                rt_timer_control(&(thread->thread_timer),
                                 RT_TIMER_CTRL_SET_TIME,
                                 &timeout);
                rt_timer_start(&(thread->thread_timer));
            }
            /* enable interrupt */
            rt_hw_interrupt_enable(level);

            /* do schedule */
            rt_schedule();

            /* thread is waked up */
            result = thread->error;

            level = rt_hw_interrupt_disable();
        }
    }
    /* clean completed flag */
    completion->flag = RT_UNCOMPLETED;

__exit:
    rt_hw_interrupt_enable(level);

    return result;
}
RTM_EXPORT(rt_completion_wait);

void rt_completion_done(struct rt_completion *completion)
{
    rt_base_t level;
    RT_ASSERT(completion != RT_NULL);

    if (completion->flag == RT_COMPLETED)
        return;

    level = rt_hw_interrupt_disable();
    completion->flag = RT_COMPLETED;

    if (!rt_list_isempty(&(completion->suspended_list)))
    {
        /* there is one thread in suspended list */
        struct rt_thread *thread;

        /* get thread entry */
        thread = rt_list_entry(completion->suspended_list.next,
                               struct rt_thread,
                               tlist);

        /* resume it */
        rt_thread_resume(thread);
        rt_hw_interrupt_enable(level);

        /* perform a schedule */
        rt_schedule();
    }
    else
    {
        rt_hw_interrupt_enable(level);
    }
}
RTM_EXPORT(rt_completion_done);


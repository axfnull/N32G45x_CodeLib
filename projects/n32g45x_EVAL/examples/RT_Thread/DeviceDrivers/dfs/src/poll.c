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
 * @file poll.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include <stdint.h>

#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>

#include <dfs.h>
#include <dfs_file.h>
#include <dfs_posix.h>
#include <dfs_poll.h>

#ifdef RT_USING_POSIX

struct rt_poll_node;

struct rt_poll_table
{
    rt_pollreq_t req;
    rt_uint32_t triggered; /* the waited thread whether triggered */
    rt_thread_t polling_thread;
    struct rt_poll_node *nodes;
};

struct rt_poll_node
{
    struct rt_wqueue_node wqn;
    struct rt_poll_table *pt;
    struct rt_poll_node *next;
};

static int __wqueue_pollwake(struct rt_wqueue_node *wait, void *key)
{
    struct rt_poll_node *pn;

    if (key && !((rt_ubase_t)key & wait->key))
        return -1;

    pn = rt_container_of(wait, struct rt_poll_node, wqn);
    pn->pt->triggered = 1;

    return __wqueue_default_wake(wait, key);
}

static void _poll_add(rt_wqueue_t *wq, rt_pollreq_t *req)
{
    struct rt_poll_table *pt;
    struct rt_poll_node *node;

    node = (struct rt_poll_node *)rt_malloc(sizeof(struct rt_poll_node));
    if (node == RT_NULL)
        return;

    pt = rt_container_of(req, struct rt_poll_table, req);

    node->wqn.key = req->_key;
    rt_list_init(&(node->wqn.list));
    node->wqn.polling_thread = pt->polling_thread;
    node->wqn.wakeup = __wqueue_pollwake;
    node->next = pt->nodes;
    node->pt = pt;
    pt->nodes = node;
    rt_wqueue_add(wq, &node->wqn);
}

static void poll_table_init(struct rt_poll_table *pt)
{
    pt->req._proc = _poll_add;
    pt->triggered = 0;
    pt->nodes = RT_NULL;
    pt->polling_thread = rt_thread_self();
}

static int poll_wait_timeout(struct rt_poll_table *pt, int msec)
{
    rt_int32_t timeout;
    int ret = 0;
    struct rt_thread *thread;
    rt_base_t level;

    thread = pt->polling_thread;

    timeout = rt_tick_from_millisecond(msec);

    level = rt_hw_interrupt_disable();

    if (timeout != 0 && !pt->triggered)
    {
        rt_thread_suspend(thread);
        if (timeout > 0)
        {
            rt_timer_control(&(thread->thread_timer),
                             RT_TIMER_CTRL_SET_TIME,
                             &timeout);
            rt_timer_start(&(thread->thread_timer));
        }

        rt_hw_interrupt_enable(level);

        rt_schedule();

        level = rt_hw_interrupt_disable();
    }

    ret = !pt->triggered;
    rt_hw_interrupt_enable(level);

    return ret;
}

static int do_pollfd(struct pollfd *pollfd, rt_pollreq_t *req)
{
    int mask = 0;
    int fd;

    fd = pollfd->fd;

    if (fd >= 0)
    {
        struct dfs_fd *f = fd_get(fd);
        mask = POLLNVAL;

        if (f)
        {
            mask = POLLMASK_DEFAULT;
            if (f->fops->poll)
            {
                req->_key = pollfd->events | POLLERR | POLLHUP;

                mask = f->fops->poll(f, req);
            }
            /* Mask out unneeded events. */
            mask &= pollfd->events | POLLERR | POLLHUP;
            fd_put(f);
        }
    }
    pollfd->revents = mask;

    return mask;
}

static int poll_do(struct pollfd *fds, nfds_t nfds, struct rt_poll_table *pt, int msec)
{
    int num;
    int istimeout = 0;
    int n;
    struct pollfd *pf;

    if (msec == 0)
    {
        pt->req._proc = RT_NULL;
        istimeout = 1;
    }

    while (1)
    {
        pf = fds;
        num = 0;

        for (n = 0; n < nfds; n ++)
        {
            if (do_pollfd(pf, &pt->req))
            {
                num ++;
                pt->req._proc = RT_NULL;
            }
            pf ++;
        }

        pt->req._proc = RT_NULL;

        if (num || istimeout)
            break;

        if (poll_wait_timeout(pt, msec))
            istimeout = 1;
    }

    return num;
}

static void poll_teardown(struct rt_poll_table *pt)
{
    struct rt_poll_node *node, *next;

    next = pt->nodes;
    while (next)
    {
        node = next;
        rt_wqueue_remove(&node->wqn);
        next = node->next;
        rt_free(node);
    }
}

int poll(struct pollfd *fds, nfds_t nfds, int timeout)
{
    int num;
    struct rt_poll_table table;

    poll_table_init(&table);

    num = poll_do(fds, nfds, &table, timeout);

    poll_teardown(&table);

    return num;
}

#endif 

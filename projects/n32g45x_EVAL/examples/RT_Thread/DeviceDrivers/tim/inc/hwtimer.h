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
 * @file hwtimer.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef __HWTIMER_H__
#define __HWTIMER_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Timer Control Command */
typedef enum
{
    HWTIMER_CTRL_FREQ_SET = 0x01,    /* set the count frequency */
    HWTIMER_CTRL_STOP,               /* stop timer */
    HWTIMER_CTRL_INFO_GET,           /* get a timer feature information */
    HWTIMER_CTRL_MODE_SET            /* Setting the timing mode(oneshot/period) */
} rt_hwtimer_ctrl_t;

/* Timing Mode */
typedef enum
{
    HWTIMER_MODE_ONESHOT = 0x01,
    HWTIMER_MODE_PERIOD
} rt_hwtimer_mode_t;

/* Time Value */
typedef struct rt_hwtimerval
{
    rt_int32_t sec;      /* second */
    rt_int32_t usec;     /* microsecond */
} rt_hwtimerval_t;

#define HWTIMER_CNTMODE_UP      0x01 /* increment count mode */
#define HWTIMER_CNTMODE_DW      0x02 /* decreasing count mode */

struct rt_hwtimer_device;

struct rt_hwtimer_ops
{
    void (*init)(struct rt_hwtimer_device *timer, rt_uint32_t state);
    rt_err_t (*start)(struct rt_hwtimer_device *timer, rt_uint32_t cnt, rt_hwtimer_mode_t mode);
    void (*stop)(struct rt_hwtimer_device *timer);
    rt_uint32_t (*count_get)(struct rt_hwtimer_device *timer);
    rt_err_t (*control)(struct rt_hwtimer_device *timer, rt_uint32_t cmd, void *args);
};

/* Timer Feature Information */
struct rt_hwtimer_info
{
    rt_int32_t maxfreq;    /* the maximum count frequency timer support */
    rt_int32_t minfreq;    /* the minimum count frequency timer support */
    rt_uint32_t maxcnt;    /* counter maximum value */
    rt_uint8_t  cntmode;   /* count mode (inc/dec) */
};

typedef struct rt_hwtimer_device
{
    struct rt_device parent;
    const struct rt_hwtimer_ops *ops;
    const struct rt_hwtimer_info *info;

    rt_int32_t freq;                /* counting frequency set by the user */
    rt_int32_t overflow;            /* timer overflows */
    float period_sec;               
    rt_int32_t cycles;              /* how many times will generate a timeout event after overflow */
    rt_int32_t reload;              /* reload cycles(using in period mode) */
    rt_hwtimer_mode_t mode;         /* timing mode(oneshot/period) */
} rt_hwtimer_t;

rt_err_t rt_device_hwtimer_register(rt_hwtimer_t *timer, const char *name, void *user_data);
void rt_device_hwtimer_isr(rt_hwtimer_t *timer);

#ifdef __cplusplus
}
#endif

#endif

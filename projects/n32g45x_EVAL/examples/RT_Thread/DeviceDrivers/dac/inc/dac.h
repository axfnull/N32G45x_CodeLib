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
 * @file dac.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef __DAC_H__
#define __DAC_H__
#include <rtthread.h>

struct rt_dac_device;
struct rt_dac_ops
{
    rt_err_t (*disabled)(struct rt_dac_device *device, rt_uint32_t channel);
    rt_err_t (*enabled)(struct rt_dac_device *device, rt_uint32_t channel);
    rt_err_t (*convert)(struct rt_dac_device *device, rt_uint32_t channel, rt_uint32_t *value);
};

struct rt_dac_device
{
    struct rt_device parent;
    const struct rt_dac_ops *ops;
};
typedef struct rt_dac_device *rt_dac_device_t;

typedef enum
{
    RT_DAC_CMD_ENABLE,
    RT_DAC_CMD_DISABLE,
} rt_dac_cmd_t;

rt_err_t rt_hw_dac_register(rt_dac_device_t dac,const char *name, const struct rt_dac_ops *ops, const void *user_data);

rt_uint32_t rt_dac_write(rt_dac_device_t dev, rt_uint32_t channel, rt_uint32_t value);
rt_err_t rt_dac_enable(rt_dac_device_t dev, rt_uint32_t channel);
rt_err_t rt_dac_disable(rt_dac_device_t dev, rt_uint32_t channel);

#endif /* __dac_H__ */

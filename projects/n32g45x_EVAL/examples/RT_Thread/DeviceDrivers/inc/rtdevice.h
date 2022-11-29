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
 * @file rtdevice.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef __RT_DEVICE_H__
#define __RT_DEVICE_H__

#include <rtthread.h>

//#include "ipc/ringbuffer.h"
//#include "ipc/completion.h"
//#include "ipc/dataqueue.h"
//#include "ipc/workqueue.h"
//#include "ipc/waitqueue.h"
//#include "ipc/pipe.h"
//#include "ipc/poll.h"
//#include "ipc/ringblk_buf.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RT_DEVICE(device)            ((rt_device_t)device)

#ifdef RT_USING_RTC
#include "rtc.h"
#ifdef RT_USING_ALARM
#include "alarm.h"
#endif
#endif /* RT_USING_RTC */

#ifdef RT_USING_SPI
#include "spi.h"
#endif /* RT_USING_SPI */

#ifdef RT_USING_MTD_NOR
#include "mtd_nor.h"
#endif /* RT_USING_MTD_NOR */

#ifdef RT_USING_MTD_NAND
#include "mtd_nand.h"
#endif /* RT_USING_MTD_NAND */

#ifdef RT_USING_USB_DEVICE
#include "usb_device.h"
#endif /* RT_USING_USB_DEVICE */

#ifdef RT_USING_USB_HOST
#include "usb_host.h"
#endif /* RT_USING_USB_HOST */

#ifdef RT_USING_SERIAL
#include "serial.h"
#endif /* RT_USING_SERIAL */

#ifdef RT_USING_I2C
#include "i2c.h"
#include "i2c_dev.h"

#ifdef RT_USING_I2C_BITOPS
#include "i2c-bit-ops.h"
#endif /* RT_USING_I2C_BITOPS */
#endif /* RT_USING_I2C */

#ifdef RT_USING_SDIO
#include "mmcsd_core.h"
#include "sd.h"
#include "sdio.h"
#endif

#ifdef RT_USING_WDT
#include "watchdog.h"
#endif

#ifdef RT_USING_PIN
#include "pin.h"
#endif

#ifdef RT_USING_CAN
#include "can.h"
#endif

#ifdef RT_USING_HWTIMER
#include "hwtimer.h"
#endif

#ifdef RT_USING_AUDIO
#include "audio.h"
#endif

#ifdef RT_USING_CPUTIME
#include "cputime.h"
#endif

#ifdef RT_USING_ADC
#include "adc.h"
#endif

#ifdef RT_USING_PWM
#include "rt_drv_pwm.h"
#endif

#ifdef RT_USING_PM
#include "pm.h"
#endif

#ifdef RT_USING_WIFI
#include "wlan.h"
#endif

#ifdef MTD_USING_NOR
#include "mtdnor.h"
#endif
#ifdef MTD_USING_NAND
#include "mtdnand.h"
#endif

#ifdef RT_USING_HWCRYPTO
#include "crypto.h"
#endif

#ifdef RT_USING_PULSE_ENCODER
#include "pulse_encoder.h"
#endif

#ifdef RT_USING_INPUT_CAPTURE
#include "rt_inputcapture.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __RT_DEVICE_H__ */

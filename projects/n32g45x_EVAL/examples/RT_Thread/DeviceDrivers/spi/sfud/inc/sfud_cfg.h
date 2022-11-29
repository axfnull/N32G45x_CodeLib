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
 * @file sfud_cfg.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef _SFUD_CFG_H_
#define _SFUD_CFG_H_

#include <rtconfig.h>

/**
 * It will print more information on debug mode.
 * #define RT_DEBUG_SFUD open debug mode */
#ifdef RT_DEBUG_SFUD
#define SFUD_DEBUG_MODE
#endif

/**
 * Using probe flash JEDEC SFDP parameter.
 */
#ifdef RT_SFUD_USING_SFDP
#define SFUD_USING_SFDP
#endif

/**
 * SFUD will support QSPI mode.
 */
#ifdef RT_SFUD_USING_QSPI
#define SFUD_USING_QSPI
#endif

/**
 * Using probe flash JEDEC ID then query defined supported flash chip information table. @see SFUD_FLASH_CHIP_TABLE
 */
#ifdef RT_SFUD_USING_FLASH_INFO_TABLE
#define SFUD_USING_FLASH_INFO_TABLE
#endif

#define SFUD_FLASH_DEVICE_TABLE {{0}}

#endif /* _SFUD_CFG_H_ */

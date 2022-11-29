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
 * @file usb_prop.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usb_prop_H
#define __usb_prop_H
#include "stdint.h"
#include "usb_core.h"

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Speaker_init(void);
void Speaker_Reset(void);
void Speaker_SetConfiguration(void);
void Speaker_SetDeviceAddress (void);
void Speaker_Status_In (void);
void Speaker_Status_Out (void);
USB_Result Speaker_Data_Setup(uint8_t);
USB_Result Speaker_NoData_Setup(uint8_t);
USB_Result Speaker_Get_Interface_Setting(uint8_t Interface, uint8_t AlternateSetting);
uint8_t *Speaker_GetDeviceDescriptor(uint16_t );
uint8_t *Speaker_GetConfigDescriptor(uint16_t);
uint8_t *Speaker_GetStringDescriptor(uint16_t);
uint8_t *Mute_Command(uint16_t Length);

/* Exported define -----------------------------------------------------------*/
#define Speaker_GetConfiguration          USB_ProcessNop
//#define Speaker_SetConfiguration          USB_ProcessNop
#define Speaker_GetInterface              USB_ProcessNop
#define Speaker_SetInterface              USB_ProcessNop
#define Speaker_GetStatus                 USB_ProcessNop
#define Speaker_ClearFeature              USB_ProcessNop
#define Speaker_SetEndPointFeature        USB_ProcessNop
#define Speaker_SetDeviceFeature          USB_ProcessNop
//#define Speaker_SetDeviceAddress          USB_ProcessNop
#define GET_CUR                           0x81
#define SET_CUR                           0x01

#endif /* __usb_prop_H */



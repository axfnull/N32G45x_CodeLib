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
 * @file usb_desc.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_DESC_H
#define __USB_DESC_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/

#define SAUDIO_SIZ_DEVICE_DESC                       18
#define AUDIO_SIZ_CONFIG_DESC                        109
#define AUDIO_SIZ_INTERFACE_DESC_SIZE                9

#define USB_DEVICE_DESCRIPTOR_TYPE                  0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE           0x02
#define USB_STRING_DESCRIPTOR_TYPE                  0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE               0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE                0x05
#define USB_SIZ_DEVICE_DESC                         18
#define USB_SIZ_STRING_LANGID                       4

#define AUDIO_SIZ_STRING_LANGID                     0x04
#define AUDIO_SIZ_STRING_VENDOR                     0x26
#define AUDIO_SIZ_STRING_PRODUCT                    0x18
#define AUDIO_SIZ_STRING_SERIAL                     0x1A

#define AUDIO_SIZ_DEVICE_DESC                       18

#define AUDIO_STANDARD_ENDPOINT_DESC_SIZE           0x09
#define AUDIO_STREAMING_ENDPOINT_DESC_SIZE          0x07

#define USB_DEVICE_CLASS_AUDIO                      0x01
#define AUDIO_SUBCLASS_AUDIOCONTROL                 0x01
#define AUDIO_SUBCLASS_AUDIOSTREAMING               0x02
#define AUDIO_PROTOCOL_UNDEFINED                    0x00
#define AUDIO_STREAMING_GENERAL                     0x01
#define AUDIO_STREAMING_FORMAT_TYPE                 0x02

/* Audio Descriptor Types */
#define AUDIO_INTERFACE_DESCRIPTOR_TYPE             0x24
#define AUDIO_ENDPOINT_DESCRIPTOR_TYPE              0x25


/* Audio Control Interface Descriptor Subtypes */
#define AUDIO_CONTROL_HEADER                        0x01
#define AUDIO_CONTROL_INPUT_TERMINAL                0x02
#define AUDIO_CONTROL_OUTPUT_TERMINAL               0x03
#define AUDIO_CONTROL_FEATURE_UNIT                  0x06

#define AUDIO_INPUT_TERMINAL_DESC_SIZE              0x0C
#define AUDIO_OUTPUT_TERMINAL_DESC_SIZE             0x09
#define AUDIO_STREAMING_INTERFACE_DESC_SIZE         0x07

#define AUDIO_CONTROL_MUTE                          0x0001

#define AUDIO_FORMAT_TYPE_I                         0x01

#define USB_ENDPOINT_TYPE_ISOCHRONOUS               0x01
#define AUDIO_ENDPOINT_GENERAL                      0x01


/* Exported functions ------------------------------------------------------- */
extern const uint8_t Audio_DeviceDescriptor[AUDIO_SIZ_DEVICE_DESC];
extern const uint8_t Audio_ConfigDescriptor[AUDIO_SIZ_CONFIG_DESC];
extern const uint8_t Audio_StringLangID[AUDIO_SIZ_STRING_LANGID];
extern const uint8_t Audio_StringVendor[AUDIO_SIZ_STRING_VENDOR];
extern const uint8_t Audio_StringProduct[AUDIO_SIZ_STRING_PRODUCT];
extern uint8_t Audio_StringSerial[AUDIO_SIZ_STRING_SERIAL];

#endif /* __USB_DESC_H */


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
 * @file usb_desc.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USB Standard Device Descriptor */
const uint8_t CustomHID_DeviceDescriptor[CUSTOMHID_SIZ_DEVICE_DESC] =
{
        0x12,                       /*bLength */
        USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
        0x00,                       /*bcdUSB */
        0x02,
        0x00,                       /*bDeviceClass*/
        0x00,                       /*bDeviceSubClass*/
        0x00,                       /*bDeviceProtocol*/
        0x40,                       /*bMaxPacketSize40*/
        0xF5,                       /*idVendor (0x19F5)*/
        0x19,
        0x50,                       /*idProduct = 0x5750*/
        0x57,
        0x00,                       /*bcdDevice rel. 2.00*/
        0x02,
        1,                          /*Index of string descriptor describing
                                                                                            manufacturer */
        2,                          /*Index of string descriptor describing
                                                                                         product*/
        3,                          /*Index of string descriptor describing the
                                                                                         device serial number */
        0x01                        /*bNumConfigurations*/
}; /* CustomHID_DeviceDescriptor */


/* USB Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
const uint8_t CustomHID_ConfigDescriptor[CUSTOMHID_SIZ_CONFIG_DESC] =
{
        0x09, /* bLength: Configuration Descriptor size */
        USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
        CUSTOMHID_SIZ_CONFIG_DESC,
        /* wTotalLength: Bytes returned */
        0x00,
        0x01,         /* bNumInterfaces: 1 interface */
        0x01,         /* bConfigurationValue: Configuration value */
        0x00,         /* iConfiguration: Index of string descriptor describing
                                                                 the configuration*/
        0xC0,         /* bmAttributes: Self powered */
        0x32,         /* MaxPower 100 mA: this current is used for detecting Vbus */

        /************** Descriptor of Custom HID interface ****************/
        /* 09 */
        0x09,         /* bLength: Interface Descriptor size */
        USB_INTERFACE_DESCRIPTOR_TYPE,/* bDescriptorType: Interface descriptor type */
        0x00,         /* bInterfaceNumber: Number of Interface */
        0x00,         /* bAlternateSetting: Alternate setting */
        0x02,         /* bNumEndpoints */
        0x03,         /* bInterfaceClass: HID */
        0x01,         /* bInterfaceSubClass : 1=BOOT, 0=no boot */
        0x01,         /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
        0,            /* iInterface: Index of string descriptor */
        /******************** Descriptor of Custom HID HID ********************/
        /* 18 */
        0x09,         /* bLength: HID Descriptor size */
        HID_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
        0x10,         /* bcdHID: HID Class Spec release number */
        0x01,
        0x00,         /* bCountryCode: Hardware target country */
        0x01,         /* bNumDescriptors: Number of HID class descriptors to follow */
        0x22,         /* bDescriptorType */
        CUSTOMHID_SIZ_REPORT_DESC,/* wItemLength: Total length of Report descriptor */
        0x00,
        /******************** Descriptor of Custom HID endpoints ******************/
        /* 27 */
        0x07,          /* bLength: Endpoint Descriptor size */
        USB_ENDPOINT_DESCRIPTOR_TYPE, /* bDescriptorType: */

        0x81,          /* bEndpointAddress: Endpoint Address (IN) */
        0x03,          /* bmAttributes: Interrupt endpoint */
        0x08,          /* wMaxPacketSize: 8 Bytes max */
        0x00,
        0x20,          /* bInterval: Polling Interval (32 ms) */
        /* 34 */
            
        0x07,   /* bLength: Endpoint Descriptor size */
        USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: */
            /*  Endpoint descriptor type */
        0x01,   /* bEndpointAddress: */
            /*  Endpoint Address (OUT) */
        0x03,   /* bmAttributes: Interrupt endpoint */
        0x02,   /* wMaxPacketSize: 2 Bytes max  */
        0x00,
        0x20,   /* bInterval: Polling Interval (20 ms) */
        /* 41 */
}; /* CustomHID_ConfigDescriptor */
const uint8_t CustomHID_ReportDescriptor[CUSTOMHID_SIZ_REPORT_DESC] =
{                    
        0x05,0x01,// Global Generic Desktop
        0x09,0x06,// Local KeyBoard 
        0xA1,0x01,// Main app collection
        0x05,0x07,// Global KeyBoard
        //////////////////////////////////?1??
        0x19,0xe0,// Local Usage Min (KeyBoard LeftControl)
        0x29,0xe7,// Local Usage Max (KeyBoard Right GUI)
        0x15,0x00,// Global Logical Min
        0x25,0x01,// Global Logical Max 
        0x95,0x08,// Global ReportCount
        0x75,0x01,// Global ReportSize
        0x81,0x02,// Main Input(Data,Var,Abs)
        //////////////////////////////////?2??
        0x95,0x01,// Global ReportCount
        0x75,0x08,// Global ReportSize
        0x81,0x03,// Main Input(Cnst,Var,Abs)
        //////////////////////////////////?3-8??
        0x95,0x06,// Global ReportCount
        0x75,0x08,// Global ReportSize
        0x15,0x00,// Global Logical Min
        0x26,0xff,0x00,//Global Logical Max
        0x19,0x00,// Local Usage Min
        0x29,0x65,// Local Usage Max
        0x81,0x00,// Main Output(Data,Ary,Abs)
        ////////////////////////////////1??????
        0x15,0x00,// Global Logical Min
        0x25,0x01,// Global Logical Max
        0x95,0x05,// Global ReportCount
        0x75,0x01,// Global ReportSize
        0x05,0x08,// Global LED
        0x19,0x01,// Local Usage Min
        0x29,0x05,// Local Usage Max
        0x91,0x02,// Main Output(Data,Var,Abs)
        ////////////////////////////////??????1???
        0x95,0x01,// Global ReportCount
        0x75,0x03,// Global ReportSize
        0x91,0x03,// Main Output(Cnst,Var,Abs)
        0xc0 // Main End collection
}; /* CustomHID_ReportDescriptor */

/* USB String Descriptors (optional) */
const uint8_t CustomHID_StringLangID[CUSTOMHID_SIZ_STRING_LANGID] =
{
        CUSTOMHID_SIZ_STRING_LANGID,
        USB_STRING_DESCRIPTOR_TYPE,
        0x09,
        0x04
}; /* LangID = 0x0409: U.S. English */

const uint8_t CustomHID_StringVendor[CUSTOMHID_SIZ_STRING_VENDOR] = {
    CUSTOMHID_SIZ_STRING_VENDOR, /* Size of Vendor string */
    USB_STRING_DESCRIPTOR_TYPE,  /* bDescriptorType*/
    /* Manufacturer: "NATIONS" */
    'N',
    0,
    'A',
    0,
    'T',
    0,
    'I',
    0,
    'O',
    0,
    'N',
    0,
    'S',
    0};

const uint8_t CustomHID_StringProduct[CUSTOMHID_SIZ_STRING_PRODUCT] = {CUSTOMHID_SIZ_STRING_PRODUCT, /* bLength */
                                                                       USB_STRING_DESCRIPTOR_TYPE, /* bDescriptorType */
                                                                       'N',
                                                                       0,
                                                                       '3',
                                                                       0,
                                                                       '2',
                                                                       0,
                                                                       'g',
                                                                       0,
                                                                       '4',
                                                                       0,
                                                                       '5',
                                                                       0,
                                                                       'x',
                                                                       0,
                                                                       'C',
                                                                       0,
                                                                       'u',
                                                                       0,
                                                                       's',
                                                                       0,
                                                                       't',
                                                                       0,
                                                                       'm',
                                                                       0,
                                                                       ' ',
                                                                       0,
                                                                       'H',
                                                                       0,
                                                                       'I',
                                                                       0,
                                                                       'D',
                                                                       0};
uint8_t CustomHID_StringSerial[CUSTOMHID_SIZ_STRING_SERIAL]         = {CUSTOMHID_SIZ_STRING_SERIAL, /* bLength */
                                                               USB_STRING_DESCRIPTOR_TYPE, /* bDescriptorType */
                                                               'N',
                                                               0,
                                                               '3',
                                                               0,
                                                               '2',
                                                               0,
                                                               'g',
                                                               0,
                                                               '4',
                                                               0,
                                                               '5',
                                                               0,
                                                               'x',
                                                               0};

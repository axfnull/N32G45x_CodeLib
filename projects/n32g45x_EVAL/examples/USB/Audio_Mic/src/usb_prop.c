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
 * @file usb_prop.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_conf.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_pwr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t MUTE_DATA = 0;

USB_Device Device_Table =
{
    EP_NUM,
    1
};

DEVICE_PROP Device_Property =
{
    Audio_init,
    Audio_Reset,
    Audio_Status_In,
    Audio_Status_Out,
    Audio_Data_Setup,
    Audio_NoData_Setup,
    Audio_Get_Interface_Setting,
    Audio_GetDeviceDescriptor,
    Audio_GetConfigDescriptor,
    Audio_GetStringDescriptor,
    0,
    0x40 /*MAX PACKET SIZE*/
};

USER_STANDARD_REQUESTS User_Standard_Requests =
{
    Audio_GetConfiguration,
    Audio_SetConfiguration,
    Audio_GetInterface,
    Audio_SetInterface,
    Audio_GetStatus,
    Audio_ClearFeature,
    Audio_SetEndPointFeature,
    Audio_SetDeviceFeature,
    Audio_SetDeviceAddress
};

USB_OneDescriptor Device_Descriptor = {(uint8_t*)Audio_DeviceDescriptor, AUDIO_SIZ_DEVICE_DESC};

USB_OneDescriptor Config_Descriptor = {(uint8_t*)Audio_ConfigDescriptor, AUDIO_SIZ_CONFIG_DESC};

USB_OneDescriptor String_Descriptor[4] = {{(uint8_t*)Audio_StringLangID, AUDIO_SIZ_STRING_LANGID},
                                          {(uint8_t*)Audio_StringVendor, AUDIO_SIZ_STRING_VENDOR},
                                          {(uint8_t*)Audio_StringProduct, AUDIO_SIZ_STRING_PRODUCT},
                                          {(uint8_t*)Audio_StringSerial, AUDIO_SIZ_STRING_SERIAL},};

/* Extern variables ----------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
extern uint16_t In_Data_Offset;
extern uint16_t Out_Data_Offset;

/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief Audio init routine.
 */
void Audio_init()
{
    /* Initialize the current configuration */
    pInformation->CurrentConfiguration = 0;

    /* Connect the device */
    PowerOn();

    /* Perform basic device initialization operations */
    USB_SilInit();

    bDeviceState = UNCONNECTED;
}

/**
 * @brief Audio reset routine.
 */
void Audio_Reset()
{
    /* Set Audio device as not configured state */
    pInformation->CurrentConfiguration = 0;

    /* Current Feature initialization */
    pInformation->CurrentFeature = Audio_ConfigDescriptor[7];

    USB_SetBuftab(BTABLE_ADDRESS);

    /* Initialize Endpoint 0 */
    USB_SetEpType(ENDP0, EP_CONTROL);
    SetEPTxStatus(ENDP0, EP_TX_NAK);
    USB_SetEpRxAddr(ENDP0, ENDP0_RXADDR);
    USB_SetEpRxCnt(ENDP0, Device_Property.MaxPacketSize);
    USB_SetEpTxAddr(ENDP0, ENDP0_TXADDR);
    USB_ClrStsOut(ENDP0);
    USB_SetEpRxValid(ENDP0);
 
    /* Initialize Endpoint 1 */
    USB_SetEpType(ENDP1, EP_ISOCHRONOUS);
    USB_SetEpDblBuferAddr(ENDP1, ENDP1_BUF0Addr, ENDP1_BUF1Addr);
    USB_ClrDattogRx(ENDP1);
    USB_ClrDattogTx(ENDP1);
    USB_DattogRx(ENDP1);

    SetEPTxStatus(ENDP1, EP_TX_VALID);
    SetEPRxStatus(ENDP1, EP_RX_DIS);

    USB_SetEpRxValid(ENDP0);
    
    /* Set this device to response on default address */
    USB_SetDeviceAddress(0);

    bDeviceState = ATTACHED;
}

/**
 * @brief Update the device state to configured.
 */
void Audio_SetConfiguration(void)
{
    USB_DeviceMess *pInfo = &Device_Info;

    if (pInfo->CurrentConfiguration != 0)
    {
        /* Device configured */
        bDeviceState = CONFIGURED;
    }
}

/**
 * @brief Update the device state to addressed.
 */
void Audio_SetDeviceAddress (void)
{
    bDeviceState = ADDRESSED;
}

/**
 * @brief Audio Status In routine.
 */
void Audio_Status_In(void)
{}

/**
 * @brief Audio Status Out routine.
 */
void Audio_Status_Out (void)
{}

/**
 * @brief Handle the data class specific requests
 * @param RequestNo£ºrequest number
 * @return  UnSupport or Success.
 */
USB_Result Audio_Data_Setup(uint8_t RequestNo)
{
    uint8_t *(*CopyRoutine)(uint16_t);
    CopyRoutine = NULL;

    if ((RequestNo == GET_CUR) || (RequestNo == SET_CUR))
    {
        CopyRoutine = Mute_Command;
    }

    else
    {
        return UnSupport;
    }

    pInformation->Ctrl_Info.CopyData = CopyRoutine;
    pInformation->Ctrl_Info.Usb_wOffset = 0;
    (*CopyRoutine)(0);
    return Success;
}

/**
 * @brief Handle the no data class specific requests.
 * @param RequestNo: request number
 * @return  UnSupport or Success.
 */
USB_Result Audio_NoData_Setup(uint8_t RequestNo)
{
    return UnSupport;
}

/**
 * @brief Get the device descriptor.
 * @param Length : data_length
 * @return The address of the device descriptor.
 */
uint8_t *Audio_GetDeviceDescriptor(uint16_t Length)
{
    return Standard_GetDescriptorData(Length, &Device_Descriptor);
}

/**
 * @brief Get the configuration descriptor.
 * @param Length : data_length
 * @return The address of the configuration descriptor.
 */
uint8_t *Audio_GetConfigDescriptor(uint16_t Length)
{
    return Standard_GetDescriptorData(Length, &Config_Descriptor);
}

/**
 * @brief Get the string descriptors according to the needed index.
 * @param Length : data_length
 * @return The address of the string descriptor.
 */
uint8_t *Audio_GetStringDescriptor(uint16_t Length)
{
    uint8_t wValue0 = pInformation->USBwValue0;

    if (wValue0 > 4)
    {
        return NULL;
    }
    else
    {
        return Standard_GetDescriptorData(Length, &String_Descriptor[wValue0]);
    }
}

/**
 * @brief test the interface and the alternate setting according to the
 *                  supported one.
 * @param Interface : interface number.
 * @param AlternateSetting : Alternate Setting number.
 * @return UnSupport or Success.
 */
USB_Result Audio_Get_Interface_Setting(uint8_t Interface, uint8_t AlternateSetting)
{
    if (AlternateSetting > 1)
    {
        return UnSupport;
    }
    else if (Interface > 1)
    {
        return UnSupport;
    }
    return Success;
}

/**
 * @brief Handle the GET MUTE and SET MUTE command.
 * @param Length : data length.
 * @return MUTE data
 */
uint8_t *Mute_Command(uint16_t Length)
{
    if (Length == 0)
    {
        pInformation->Ctrl_Info.Usb_wLength = pInformation->wLengths.w;
        return NULL;
    }
    else
    {
        return((uint8_t*)(&MUTE_DATA));
    }
}

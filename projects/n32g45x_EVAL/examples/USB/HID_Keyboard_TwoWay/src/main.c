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
 * @file main.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "n32g45x.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_pwr.h"
__IO uint8_t PrevXferComplete = 1;
__IO uint32_t TimingDelay     = 0;
u8 key_buffer[8]              = {0};
u8* Ep1DataPtr                = 0;
extern u8* EpOutDataPtrTmp;
extern u8* EpInDataPtrTmp;

/**
 * @brief  Main program.
 */
int main(void)
{
    Set_System();

    USB_Interrupts_Config();

    Set_USBClock();

    USB_Init();

    while (bDeviceState != CONFIGURED);

    while (1)
    {
        if (PrevXferComplete)
        {
            PrevXferComplete = 0;
            if (!GPIO_ReadInputDataBit(KEY_PORT, KEY_A_PIN))
            {
                key_buffer[2] = 0x04;
            }
            else if (!GPIO_ReadInputDataBit(KEY_PORT, KEY_B_PIN))
            {
                key_buffer[2] = 0x05;
            }
            else if (!GPIO_ReadInputDataBit(KEY_PORT, KEY_C_PIN))
            {
                key_buffer[2] = 0x06;
            }
            Ep1DataPtr       = key_buffer;
            USB_SilWrite(EP1_IN, Ep1DataPtr, 8);
            Ep1DataPtr    = EpInDataPtrTmp;
            key_buffer[2] = 0;
        }    
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
* @param   expr: If expr is false, it calls assert_failed function which reports
 *         the name of the source file and the source line number of the call
 *         that failed. If expr is true, it returns no value.
 * @param  file: pointer to the source file name.
 * @param  line: assert_param error line source number.
 */
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

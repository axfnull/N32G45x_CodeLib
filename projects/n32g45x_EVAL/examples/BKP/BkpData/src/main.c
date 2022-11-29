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
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "main.h"

/** @addtogroup N32G45X_StdPeriph_Examples
 * @{
 */

/** @addtogroup BkpData
 * @{
 */

#define BKP_DR_NUMBER 42
#define LED1          GPIO_PIN_4
#define LED2          GPIO_PIN_5
#define LED3          GPIO_PIN_6

uint16_t BKPDataReg[BKP_DR_NUMBER] = {BKP_DAT1,  BKP_DAT2,  BKP_DAT3,  BKP_DAT4,  BKP_DAT5,  BKP_DAT6,  BKP_DAT7,
                                      BKP_DAT8,  BKP_DAT9,  BKP_DAT10, BKP_DAT11, BKP_DAT12, BKP_DAT13, BKP_DAT14,
                                      BKP_DAT15, BKP_DAT16, BKP_DAT17, BKP_DAT18, BKP_DAT19, BKP_DAT20, BKP_DAT21,
                                      BKP_DAT22, BKP_DAT23, BKP_DAT24, BKP_DAT25, BKP_DAT26, BKP_DAT27, BKP_DAT28,
                                      BKP_DAT29, BKP_DAT30, BKP_DAT31, BKP_DAT32, BKP_DAT33, BKP_DAT34, BKP_DAT35,
                                      BKP_DAT36, BKP_DAT37, BKP_DAT38, BKP_DAT39, BKP_DAT40, BKP_DAT41, BKP_DAT42};

void WriteToBackupReg(uint16_t FirstBackupData);
uint8_t CheckBackupReg(uint16_t FirstBackupData);

/**
 * @brief  Configures LED GPIO.
 * @param Led Specifies the Led to be configured.
 *   This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedInit(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_InitType GPIO_InitStructure;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

    /* Enable the GPIO Clock */
    if (GPIOx == GPIOA)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    }
    else if (GPIOx == GPIOB)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    }
    else if (GPIOx == GPIOC)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC, ENABLE);
    }
    else if (GPIOx == GPIOD)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOD, ENABLE);
    }
    else if (GPIOx == GPIOE)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOE, ENABLE);
    }
    else if (GPIOx == GPIOF)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOF, ENABLE);
    }
    else
    {
        if (GPIOx == GPIOG)
        {
            RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOG, ENABLE);
        }
    }

    /* Configure the GPIO pin */
//    if (Pin <= GPIO_PIN_ALL)
    {
        GPIO_InitStructure.Pin        = Pin & GPIO_PIN_ALL;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);
        // GPIOx->PBSC = Pin;
    }
}

/**
 * @brief  Turns selected Led on.
 * @param Led Specifies the Led to be set on.
 */
void LedOn(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->PBSC = Pin;
}
/**
 * @brief  Turns selected Led Off.
 * @param Led Specifies the Led to be set off.
 */
void LedOff(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->PBC = Pin;
}

/**
 * @brief  Toggles the selected Led.
 */
void LedBlink(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->POD ^= Pin;
}

/**
 * @brief  Main program.
 */
int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured,
         this is done through SystemInit() function which is called from startup
         file (startup_n32g45x.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_n32g45x.c file
       */
    LedInit(GPIOE, LED1 | LED2 | LED3);
    LedOff(GPIOE, LED1 | LED2 | LED3);

    /* Enable PWR and BKP clock */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR | RCC_APB1_PERIPH_BKP, ENABLE);

    /* Enable write access to Backup domain */
    PWR_BackupAccessEnable(ENABLE);

    /* Clear Tamper pin Event(TE) pending flag */
    BKP_ClrTEFlag();

    /* Check if the Power On Reset flag is set */
   // if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
    {
        /* Clear reset flags */
        RCC_ClrFlag();

        /* Turn on LED3 */
        LedOn(GPIOE, LED3);
        /* Check if Backup data registers are programmed */
        if (CheckBackupReg(0x3210) == 0x00)
        { /* Backup data registers values are correct */

            /* Turn on LED1 */
            LedOn(GPIOE, LED1);
        }
        else
        { /* Backup data registers values are not correct or they are not yet
             programmed (when the first time the program is executed) */

            /* Write data to Backup data registers */
            WriteToBackupReg(0x3210);

            /* Turn on LED2 */
            LedOn(GPIOE, LED2);
        }
    }
    while (1)
    {
    }
}

/**
 * @brief  Writes data Backup DRx registers.
 * @param FirstBackupData data to be written to Backup data registers.
 */
void WriteToBackupReg(uint16_t FirstBackupData)
{
    uint32_t index = 0;

    for (index = 0; index < BKP_DR_NUMBER; index++)
    {
        BKP_WriteBkpData(BKPDataReg[index], FirstBackupData + (index * 0x5A));
    }
}

/**
 * @brief  Checks if the Backup DRx registers values are correct or not.
 * @param FirstBackupData data to be compared with Backup data registers.
 * @return
 *          - 0: All Backup DRx registers values are correct
 *          - Value different from 0: Number of the first Backup register
 *            which value is not correct
 */
uint8_t CheckBackupReg(uint16_t FirstBackupData)
{
    uint32_t index = 0;

    for (index = 0; index < BKP_DR_NUMBER; index++)
    {
        if (BKP_ReadBkpData(BKPDataReg[index]) != (FirstBackupData + (index * 0x5A)))
        {
            return (index + 1);
        }
    }

    return 0;
}

#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param file pointer to the source file name
 * @param line assert_param error line source number
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

/**
 * @}
 */

/**
 * @}
 */

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
 * @file usb_pwr.c
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "usb_lib.h"
#include "usb_conf.h"
#include "usb_pwr.h"
#include "hw_config.h"

__IO uint32_t bDeviceState = UNCONNECTED; /* USB device status */
__IO bool fSuspendEnabled  = true;        /* true when suspend is possible */
__IO uint32_t EP[8];

struct
{
    __IO RESUME_STATE eState;
    __IO uint8_t bESOFcnt;
} ResumeS;

__IO uint32_t remotewakeupon = 0;

/* Extern function prototypes ------------------------------------------------*/

/**
 * @brief PowerOn
 * @return Success.
 */
USB_Result PowerOn(void)
{
    uint16_t wRegVal;

    /*** CNTR_PWDN = 0 ***/
    wRegVal = CTRL_FRST;
    _SetCNTR(wRegVal);

    /*** CTRL_FRST = 0 ***/
    wInterrupt_Mask = 0;
    _SetCNTR(wInterrupt_Mask);
    /*** Clear pending interrupts ***/
    _SetISTR(0);
    /*** Set interrupt mask ***/
    wInterrupt_Mask = CTRL_RSTM | CTRL_SUSPDM | CTRL_WKUPM;
    _SetCNTR(wInterrupt_Mask);

    return Success;
}

/**
 * @brief handles switch-off conditions
 * @return Success.
 */
USB_Result PowerOff()
{
    /* disable all interrupts and force USB reset */
    _SetCNTR(CTRL_FRST);
    /* clear interrupt status register */
    _SetISTR(0);
    /* switch-off device */
    _SetCNTR(CTRL_FRST + CTRL_PD);
    /* sw variables reset */
    /* ... */

    return Success;
}
#ifdef USB_LOW_PWR_MGMT_SUPPORT
/**
* @brief  Configures system clock after wake-up from STOP: enable HSE, PLL
*         and select PLL as system clock source.
*/
void SYSCLKConfig_STOP(uint32_t RCC_PLLMULL)
{
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/
    /* Enable HSE */
    RCC->CTRL |= ((uint32_t)RCC_CTRL_HSEEN);
    /* Wait till HSE is ready and if Time out is reached exit */
    do
    {
        HSEStatus = RCC->CTRL & RCC_CTRL_HSERDF;
        StartUpCounter++;
    } while ((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));
    if ((RCC->CTRL & RCC_CTRL_HSERDF) != RESET)
    {
        HSEStatus = (uint32_t)0x01;
    }
    else
    {
        HSEStatus = (uint32_t)0x00;
    }

    if (HSEStatus == (uint32_t)0x01)
    {
        /* Enable Prefetch Buffer */
        FLASH->AC |= FLASH_AC_PRFTBFEN;

        /* Flash 2 wait state */
        FLASH->AC &= (uint32_t)((uint32_t)~FLASH_AC_LATENCY);
        FLASH->AC |= (uint32_t)FLASH_AC_LATENCY_4;
        /* HCLK = SYSCLK */
        RCC->CFG |= (uint32_t)RCC_CFG_AHBPRES_DIV1;
        /* PCLK2 = HCLK */
        RCC->CFG |= (uint32_t)RCC_CFG_APB2PRES_DIV2; // RCC_CFG_APB2PRES_DIV1
        /* PCLK1 = HCLK */
        RCC->CFG |= (uint32_t)RCC_CFG_APB1PRES_DIV4; // RCC_CFG_APB1PRES_DIV2
        /*  PLL configuration: PLLCLK = HSE * 18 = 144 MHz */
        RCC->CFG &= (uint32_t)((uint32_t) ~(RCC_CFG_PLLSRC | RCC_CFG_PLLHSEPRES | RCC_CFG_PLLMULFCT));
        RCC->CFG |= (uint32_t)(RCC_CFG_PLLSRC_HSE | RCC_PLLMULL);
        /* Enable PLL */
        RCC->CTRL |= RCC_CTRL_PLLEN;
        /* Wait till PLL is ready */
        while ((RCC->CTRL & RCC_CTRL_PLLRDF) == 0)
        {
        }
        /* Select PLL as system clock source */
        RCC->CFG &= (uint32_t)((uint32_t) ~(RCC_CFG_SCLKSW));
        RCC->CFG |= (uint32_t)RCC_CFG_SCLKSW_PLL;
        /* Wait till PLL is used as system clock source */
        while ((RCC->CFG & (uint32_t)RCC_CFG_SCLKSTS) != (uint32_t)0x08)
        {
        }
    }
    else
    { /* If HSE fails to start-up, the application will have wrong clock
        configuration. User can add here some code to deal with this error */
    }
}
#endif  /* USB_LOW_PWR_MGMT_SUPPORT */

/*******************************************************************************
 * Function Name  : Suspend
 * Description    : sets suspend mode operating conditions
 * Input          : None.
 * Output         : None.
 * Return         : Success.
 *******************************************************************************/
void Suspend(void)
{
    uint32_t i = 0;
    uint16_t wCNTR;
        
    /* suspend preparation */
    /* ... */

    /*Store CTRL value */
    wCNTR = _GetCNTR();

    /* This a sequence to apply a force RESET to handle a robustness case */

    /*Store endpoints registers status */
    for (i = 0; i < 8; i++)
        EP[i] = _GetENDPOINT(i);

    /* unmask RESET flag */
    wCNTR |= CTRL_RSTM;
    _SetCNTR(wCNTR);

    /*apply FRES */
    wCNTR |= CTRL_FRST;
    _SetCNTR(wCNTR);

    /*clear FRES*/
    wCNTR &= ~CTRL_FRST;
    _SetCNTR(wCNTR);

    /*poll for RESET flag in STS*/
    while ((_GetISTR() & STS_RST) == 0)
        ;

    /* clear RESET flag in STS */
    _SetISTR((uint16_t)CLR_RST);

    /*restore Enpoints*/
    for (i = 0; i < 8; i++)
        _SetENDPOINT(i, EP[i]);

    /* Now it is safe to enter macrocell in suspend mode */
    wCNTR |= CTRL_FSUSPD;
    _SetCNTR(wCNTR);

    /* force low-power mode in the macrocell */
    wCNTR = _GetCNTR();
    wCNTR |= CTRL_LP_MODE;
    _SetCNTR(wCNTR);

#ifdef USB_LOW_PWR_MGMT_SUPPORT
    PWR_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
    SYSCLKConfig_STOP(RCC_CFG_PLLMULFCT18);
#endif  /* USB_LOW_PWR_MGMT_SUPPORT */
}

/*******************************************************************************
 * Function Name  : Resume_Init
 * Description    : Handles wake-up restoring normal operations
 * Input          : None.
 * Output         : None.
 * Return         : Success.
 *******************************************************************************/
void Resume_Init(void)
{
    uint16_t wCNTR;

    /* ------------------ ONLY WITH BUS-POWERED DEVICES ---------------------- */
    /* restart the clocks */
    /* ...  */

    /* CTRL_LP_MODE = 0 */
    wCNTR = _GetCNTR();
    wCNTR &= (~CTRL_LP_MODE);
    _SetCNTR(wCNTR);

#ifdef USB_LOW_PWR_MGMT_SUPPORT      
    /* restore full power */
    /* ... on connected devices */
    Leave_LowPowerMode();
#endif /* USB_LOW_PWR_MGMT_SUPPORT */

    /* reset FSUSP bit */
    _SetCNTR(IMR_MSK);

    /* reverse suspend preparation */
    /* ... */
}

/**
 * @brief  This is the state machine handling resume operations and
 *                 timing sequence. The control is based on the Resume structure
 *                 variables and on the ESOF interrupt calling this subroutine
 *                 without changing machine state.
* @param eResumeSetVal: a state machine value (RESUME_STATE)
 *                  RESUME_ESOF doesn't change ResumeS.eState allowing
 *                  decrementing of the ESOF counter in different states.
 */
void Resume(RESUME_STATE eResumeSetVal)
{
    uint16_t wCNTR;

    if (eResumeSetVal != RESUME_ESOF)
        ResumeS.eState = eResumeSetVal;
    switch (ResumeS.eState)
    {
    case RESUME_EXTERNAL:
        if (remotewakeupon == 0)
        {
            Resume_Init();
            ResumeS.eState = RESUME_OFF;
        }
        else /* RESUME detected during the RemoteWAkeup signalling => keep RemoteWakeup handling*/
        {
            ResumeS.eState = RESUME_ON;
        }
        break;
    case RESUME_INTERNAL:
        Resume_Init();
        ResumeS.eState = RESUME_START;
        remotewakeupon = 1;
        break;
    case RESUME_LATER:
        ResumeS.bESOFcnt = 2;
        ResumeS.eState   = RESUME_WAIT;
        break;
    case RESUME_WAIT:
        ResumeS.bESOFcnt--;
        if (ResumeS.bESOFcnt == 0)
            ResumeS.eState = RESUME_START;
        break;
    case RESUME_START:
        wCNTR = _GetCNTR();
        wCNTR |= CTRL_RESUM;
        _SetCNTR(wCNTR);
        ResumeS.eState   = RESUME_ON;
        ResumeS.bESOFcnt = 10;
        break;
    case RESUME_ON:
        ResumeS.bESOFcnt--;
        if (ResumeS.bESOFcnt == 0)
        {
            wCNTR = _GetCNTR();
            wCNTR &= (~CTRL_RESUM);
            _SetCNTR(wCNTR);
            ResumeS.eState = RESUME_OFF;
            remotewakeupon = 0;
        }
        break;
    case RESUME_OFF:
    case RESUME_ESOF:
    default:
        ResumeS.eState = RESUME_OFF;
        break;
    }
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

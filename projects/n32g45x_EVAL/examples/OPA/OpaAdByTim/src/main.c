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
 * @version v1.0.3
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "main.h"

/** @addtogroup OPA_PGA
 * @{
 */

ADC_InitType ADC_InitStructure;
DMA_InitType DMA_InitStructure;
uint8_t gRunFlag[4] = {0};

void RCC_Configuration(void);
void GPIO_Configuration(void);
void OPA_Configuration(void);
void COMP_Configuratoin(void);
void ADC_SampleConfig(void);
void TIM_PwmConfig(TIM_Module* TIMx);
void TIM_AllPwmOpen(TIM_Module* TIMx);
void TIM_AllPwmShut(TIM_Module* TIMx);
void TIM_DutySet(TIM_Module* TIMx, int16_t duty1, int16_t duty2, int16_t duty3);
void TIM_Run(TIM_Module* TIMx, uint8_t* RunFlag);

/**
 * @brief   Main program,Test PGA is work ok? Opa out Pin can view by scope
 */
uint8_t Gain = 0, BakGain = 0;
int main(void)
{
    /* System clocks configuration ---------------------------------------------*/
    RCC_Configuration();

    /* GPIO configuration ------------------------------------------------------*/
    GPIO_Configuration();

    /* OPA configuration ------------------------------------------------------*/
    OPA_Configuration();

    /* ADC configuration ------------------------------------------------------*/
    ADC_SampleConfig();

    /* COMP configuration ------------------------------------------------------*/
    COMP_Configuratoin();

    /* TIMx configuration ------------------------------------------------------*/
    TIM_PwmConfig(TIM1);
    TIM_PwmConfig(TIM8);

    /*test*/
    TIM_AllPwmOpen(TIM1);
    TIM_AllPwmOpen(TIM8);
    while (1)
    {
        TIM_Run(TIM1, &gRunFlag[0]);
        TIM_Run(TIM8, &gRunFlag[1]);
        if (BakGain != Gain)
        {
            BakGain = Gain;
            switch (Gain)
            {
            case 2:
                OPAMP_SetPgaGain(OPAMP1, OPAMP_CS_PGA_GAIN_2);
                OPAMP_SetPgaGain(OPAMP2, OPAMP_CS_PGA_GAIN_2);
                OPAMP_SetPgaGain(OPAMP3, OPAMP_CS_PGA_GAIN_2);
                OPAMP_SetPgaGain(OPAMP4, OPAMP_CS_PGA_GAIN_2);
                break;
            case 4:
                OPAMP_SetPgaGain(OPAMP1, OPAMP_CS_PGA_GAIN_4);
                OPAMP_SetPgaGain(OPAMP2, OPAMP_CS_PGA_GAIN_4);
                OPAMP_SetPgaGain(OPAMP3, OPAMP_CS_PGA_GAIN_4);
                OPAMP_SetPgaGain(OPAMP4, OPAMP_CS_PGA_GAIN_4);
                break;
            case 8:
                OPAMP_SetPgaGain(OPAMP1, OPAMP_CS_PGA_GAIN_8);
                OPAMP_SetPgaGain(OPAMP2, OPAMP_CS_PGA_GAIN_8);
                OPAMP_SetPgaGain(OPAMP3, OPAMP_CS_PGA_GAIN_8);
                OPAMP_SetPgaGain(OPAMP4, OPAMP_CS_PGA_GAIN_8);
                break;
            case 16:
                OPAMP_SetPgaGain(OPAMP1, OPAMP_CS_PGA_GAIN_16);
                OPAMP_SetPgaGain(OPAMP2, OPAMP_CS_PGA_GAIN_16);
                OPAMP_SetPgaGain(OPAMP3, OPAMP_CS_PGA_GAIN_16);
                OPAMP_SetPgaGain(OPAMP4, OPAMP_CS_PGA_GAIN_16);
                break;
            case 32:
                OPAMP_SetPgaGain(OPAMP1, OPAMP_CS_PGA_GAIN_32);
                OPAMP_SetPgaGain(OPAMP2, OPAMP_CS_PGA_GAIN_32);
                OPAMP_SetPgaGain(OPAMP3, OPAMP_CS_PGA_GAIN_32);
                OPAMP_SetPgaGain(OPAMP4, OPAMP_CS_PGA_GAIN_32);
                break;
            default:
                break;
            }
        }
    }
}

/**
 * @brief  Start Demo.
 */
void TIM_Run(TIM_Module* TIMx, uint8_t* RunFlag)
{
    if (*RunFlag == 1)
    {
        TIM_AllPwmOpen(TIMx);
        *RunFlag = 0;
    }
    if (*RunFlag == 2)
    {
        TIM_AllPwmShut(TIMx);
        *RunFlag = 0;
    }
}

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* Enable GPIOA, GPIOB, GPIOC and GPIOD clocks */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO | RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_GPIOC
                                | RCC_APB2_PERIPH_GPIOD | RCC_APB2_PERIPH_GPIOE | RCC_APB2_PERIPH_GPIOF,
                            ENABLE);

    /*only support swd*/
    GPIO_ConfigPinRemap(GPIO_RMP_SW_JTAG_SW_ENABLE, ENABLE);

    /* Enable COMP OPA clocks */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_COMP | RCC_APB1_PERIPH_OPAMP | RCC_APB1_PERIPH_COMP_FILT, ENABLE);

    /* Enable ADC1, ADC2, ADC3 and ADC4 clocks */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC1 | RCC_AHB_PERIPH_ADC2 | RCC_AHB_PERIPH_ADC3 | RCC_AHB_PERIPH_ADC4,
                           ENABLE);
    /* RCC_ADCHCLK_DIV16*/
    RCC_ConfigAdcHclk(RCC_ADCHCLK_DIV10);

    /* Enable TIMx clocks */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_TIM1 | RCC_APB2_PERIPH_TIM8, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 */
static uint8_t swit = 0;
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    /*OPA VP     OPA1_VP, OPA2_Vp,  (OPA12)        OPA3_Vp, OPA4_Vp     (OP34)
                 PA5      PB0        PA7            PC9      PC5         PC3        as analog inputs */
    OPAMP_SetVmSel(OPAMP1, OPAMPx_CS_VMSEL_FLOAT);
    OPAMP_SetVmSel(OPAMP2, OPAMPx_CS_VMSEL_FLOAT);
    OPAMP_SetVmSel(OPAMP3, OPAMPx_CS_VMSEL_FLOAT);
    OPAMP_SetVmSel(OPAMP4, OPAMPx_CS_VMSEL_FLOAT);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    if (swit == 0)
    { // My Demo Need Cfg
        OPAMP_SetVpSel(OPAMP1, OPAMP1_CS_VPSEL_DAC2_PA5);
        OPAMP_SetVpSel(OPAMP2, OPAMP2_CS_VPSEL_PB0);
        OPAMP_SetVpSel(OPAMP3, OPAMP3_CS_VPSEL_PC9);
        OPAMP_SetVpSel(OPAMP4, OPAMP4_CS_VPSEL_PC5);
        GPIO_InitStructure.Pin = GPIO_PIN_5 | GPIO_PIN_7;
        GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
        GPIO_InitStructure.Pin = GPIO_PIN_0;
        GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
        GPIO_InitStructure.Pin = GPIO_PIN_9 | GPIO_PIN_5 | GPIO_PIN_3;
        GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
    }
    else if (swit == 1)
    { // PA1    PA7 PC9 PC3
        OPAMP_SetVpSel(OPAMP1, OPAMP1_CS_VPSEL_PA1);
        OPAMP_SetVpSel(OPAMP2, OPAMP2_CS_VPSEL_PA7);
        OPAMP_SetVpSel(OPAMP3, OPAMP3_CS_VPSEL_PC9);
        OPAMP_SetVpSel(OPAMP4, OPAMP4_CS_VPSEL_PC3);
        GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_7;
        GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
        GPIO_InitStructure.Pin = GPIO_PIN_9 | GPIO_PIN_3;
        GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
    }
    else if (swit == 2)
    { // PA3    PB0 PA1 DAC1/PA4
        OPAMP_SetVpSel(OPAMP1, OPAMP1_CS_VPSEL_PA3);
        OPAMP_SetVpSel(OPAMP2, OPAMP2_CS_VPSEL_PB0);
        OPAMP_SetVpSel(OPAMP3, OPAMP3_CS_VPSEL_PA1);
        OPAMP_SetVpSel(OPAMP4, OPAMP4_CS_VPSEL_DAC1_PA4);
        GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_1 | GPIO_PIN_4;
        GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
        GPIO_InitStructure.Pin = GPIO_PIN_0;
        GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
    }
    else if (swit == 3)
    { // DAC2/PA5   PE8 DAC2/PA5    PC5
        OPAMP_SetVpSel(OPAMP1, OPAMP1_CS_VPSEL_DAC2_PA5);
        OPAMP_SetVpSel(OPAMP2, OPAMP2_CS_VPSEL_PE8);
        OPAMP_SetVpSel(OPAMP3, OPAMP3_CS_VPSEL_DAC2_PA5);
        OPAMP_SetVpSel(OPAMP4, OPAMP4_CS_VPSEL_PC5);
        GPIO_InitStructure.Pin = GPIO_PIN_5;
        GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
        GPIO_InitStructure.Pin = GPIO_PIN_5;
        GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
        GPIO_InitStructure.Pin = GPIO_PIN_8;
        GPIO_InitPeripheral(GPIOE, &GPIO_InitStructure);
    }
    else if (swit == 4)
    { // PC3    xx  PC3 xx
        OPAMP_SetVpSel(OPAMP1, OPAMP1_CS_VPSEL_PA7);
        OPAMP_SetVpSel(OPAMP2, OPAMP2_CS_VPSEL_PB0);
        OPAMP_SetVpSel(OPAMP3, OPAMP3_CS_VPSEL_PC3);
        OPAMP_SetVpSel(OPAMP4, OPAMP4_CS_VPSEL_PC5);
        GPIO_InitStructure.Pin = GPIO_PIN_0;
        GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
        GPIO_InitStructure.Pin = GPIO_PIN_5 | GPIO_PIN_3;
        GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
    }
    else if (swit == 5)
    { // PA7    xx  xx  xx
        OPAMP_SetVpSel(OPAMP1, OPAMP1_CS_VPSEL_PA7);
        OPAMP_SetVpSel(OPAMP2, OPAMP2_CS_VPSEL_PB0);
        OPAMP_SetVpSel(OPAMP3, OPAMP3_CS_VPSEL_PC9);
        OPAMP_SetVpSel(OPAMP4, OPAMP4_CS_VPSEL_PC5);
        GPIO_InitStructure.Pin = GPIO_PIN_7;
        GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
        GPIO_InitStructure.Pin = GPIO_PIN_0;
        GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
        GPIO_InitStructure.Pin = GPIO_PIN_9 | GPIO_PIN_5;
        GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
    }
    /*OPA output pin enable pix pin when OPAx En.not to remap or select output pin*/
    /*OPA OUT       OP1_out,  OP2_out,             OP3_out,  OP4_out,
                    PA6        PB1                 PB11      PB12       as analog output */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.Pin       = GPIO_PIN_6;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    /*ADC port config VBUS    TEMP
                   PA0     PC0               */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.Pin       = GPIO_PIN_0;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_0;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);

    /*COMP Config CP1_INP,    CP1_OUT(remap 'b11),   CP7_INP,  CP7_OUT
                   PA1         PB8                   PC1       PC2         */
    GPIO_ConfigPinRemap(GPIO_RMP3_COMP1, ENABLE);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.Pin        = GPIO_PIN_1;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_1;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.Pin = GPIO_PIN_8;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_2;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);

    /*TIM1 for FOC   CH1,   CH1,   CH1,    CH1N,   CH2N,   CH3N
                     PA8    PA9    PA10    PB13    PB14    PB15      */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin        = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
    /*TIM8 for FOC   CH1,   CH1,   CH1,   CH1N,   CH2N,   CH3N
                     PC6    PC7    PC8    PA15    PC12    PD2     remap 1 */
    GPIO_ConfigPinRemap(GPIO_RMP1_TIM8, ENABLE);
    GPIO_InitStructure.Pin        = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.Pin       = GPIO_PIN_15;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_12;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_2;
    GPIO_InitPeripheral(GPIOD, &GPIO_InitStructure);
}

/**
 * @brief  Configures the Opa.
 */
void OPA_Configuration(void)
{
    OPAMP_InitType OPAMP_Initial;
    OPAMP_StructInit(&OPAMP_Initial);
    OPAMP_Initial.Mod            = OPAMP_CS_FOLLOW; // OPA_CS_FOLLOW;//OPA_CS_PGA_EN;
    OPAMP_Initial.Gain           = OPAMP_CS_PGA_GAIN_2;
    OPAMP_Initial.HighVolRangeEn = ENABLE;
    OPAMP_Initial.TimeAutoMuxEn  = DISABLE;
    /*configure opa1*/
    OPAMP_Init(OPAMP1, &OPAMP_Initial);
    OPAMP_Enable(OPAMP1, ENABLE);
    /*configure opa2*/
    OPAMP_Init(OPAMP2, &OPAMP_Initial);
    OPAMP_Enable(OPAMP2, ENABLE);
    /*configure opa3*/
    OPAMP_Init(OPAMP3, &OPAMP_Initial);
    OPAMP_Enable(OPAMP3, ENABLE);
    /*configure opa4*/
    OPAMP_Init(OPAMP4, &OPAMP_Initial);
    OPAMP_Enable(OPAMP4, ENABLE);
}

/**
 * @brief  Configures the comp module,this use comp1 and comp7.
 */
void COMP_Configuratoin(void)
{
    COMP_InitType COMP_Initial;

    /*Set ref1 and ref 2 voltage div*/
    COMP_SetRefScl(32, true, 32, true);

    /*Initial comp1 and comp7*/
    COMP_StructInit(&COMP_Initial);
    COMP_Initial.InpSel     = COMP1_CTRL_INPSEL_PA1;
    COMP_Initial.InmSel     = COMP1_CTRL_INMSEL_VERF1;
    COMP_Initial.SampWindow = 50;
    COMP_Initial.Thresh     = 30;
    COMP_Init(COMP1, &COMP_Initial);
    COMP_Initial.InpSel = COMP7_CTRL_INPSEL_PC1;
    COMP_Initial.InmSel = COMP7_CTRL_INMSEL_VERF2;
    COMP_Init(COMP7, &COMP_Initial);

    /*trig initial as tim1&tim8 break*/
    COMP_SetOutTrig(COMP1, COMP1_CTRL_OUTSEL_TIM1_BKIN_TIM8_BKIN);
    COMP_SetOutTrig(COMP7, COMP7_CTRL_OUTSEL_TIM1_BKIN_TIM8_BKIN);

    /*enable comp1*/
    COMP_Enable(COMP1, ENABLE);
    COMP_Enable(COMP7, ENABLE);
}

/**
 * @brief  Configures the Adcx.
 */
void ADC_SampleConfig(void)
{
    ADC_InitType ADC_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    /* ADCx registers reenable */
    ADC_DeInit(ADC1);
    ADC_DeInit(ADC2);
    ADC_DeInit(ADC3);
    ADC_DeInit(ADC4);

    /*ADCx configuration*/
    ADC_InitStruct(&ADC_InitStructure);
    ADC_InitStructure.WorkMode       = ADC_WORKMODE_INJ_SIMULT;
    ADC_InitStructure.MultiChEn      = ENABLE;
    ADC_InitStructure.ContinueConvEn = DISABLE;
    ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIG_INJ_CONV_T1_CC4;
    ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber      = 0;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Init(ADC2, &ADC_InitStructure);
    ADC_InitStructure.ExtTrigSelect = ADC_EXT_TRIG_INJ_CONV_T8_CC4;
    ADC_Init(ADC3, &ADC_InitStructure);
    ADC_Init(ADC4, &ADC_InitStructure);

    /*ADCx Injected conversions configuration,Config Sampling Time*/
    ADC_ConfigInjectedSequencerLength(ADC1, 2);
    ADC_ConfigInjectedChannel(ADC1, ADC_CH_3, 1, ADC_SAMP_TIME_1CYCLES5);
    ADC_ConfigInjectedChannel(ADC1, ADC_CH_16, 2, ADC_SAMP_TIME_239CYCLES5);
    ADC_ConfigInjectedSequencerLength(ADC2, 2);
    ADC_ConfigInjectedChannel(ADC2, ADC_CH_3, 1, ADC_SAMP_TIME_1CYCLES5);
    ADC_ConfigInjectedChannel(ADC2, ADC_CH_18, 2, ADC_SAMP_TIME_239CYCLES5);
    ADC_ConfigInjectedSequencerLength(ADC3, 2);
    ADC_ConfigInjectedChannel(ADC3, ADC_CH_1, 1, ADC_SAMP_TIME_1CYCLES5);
    ADC_ConfigInjectedChannel(ADC3, ADC_CH_18, 2, ADC_SAMP_TIME_239CYCLES5);
    ADC_ConfigInjectedSequencerLength(ADC4, 2);
    ADC_ConfigInjectedChannel(ADC4, ADC_CH_3, 1, ADC_SAMP_TIME_1CYCLES5);
    ADC_ConfigInjectedChannel(ADC4, ADC_CH_18, 2, ADC_SAMP_TIME_239CYCLES5);

    /*ADC1 and ADC2 TrigInJectConv Enable*/
    ADC_EnableExternalTrigInjectedConv(ADC1, ENABLE);
    ADC_EnableExternalTrigInjectedConv(ADC2, ENABLE);
    ADC_EnableExternalTrigInjectedConv(ADC3, ENABLE);
    ADC_EnableExternalTrigInjectedConv(ADC4, ENABLE);

    /*NVIC Initial*/
    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    /*Enable the ADC Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = ADC3_4_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    /*ADCx enable*/
    ADC_Enable(ADC1, ENABLE);
    ADC_Enable(ADC2, ENABLE);
    ADC_Enable(ADC3, ENABLE);
    ADC_Enable(ADC4, ENABLE);
        
    /*Check ADC Ready*/
    while(ADC_GetFlagStatusNew(ADC1,ADC_FLAG_RDY) == RESET);
    while(ADC_GetFlagStatusNew(ADC2,ADC_FLAG_RDY) == RESET);
    while(ADC_GetFlagStatusNew(ADC3,ADC_FLAG_RDY) == RESET);
    while(ADC_GetFlagStatusNew(ADC4,ADC_FLAG_RDY) == RESET);
            
    /*Start Calibration*/
    ADC_StartCalibration(ADC1);
    ADC_StartCalibration(ADC2);
    ADC_StartCalibration(ADC3);
    ADC_StartCalibration(ADC4);
    /* Wait for the end of ADCs calibration */
    while (ADC_GetCalibrationStatus(ADC1) & ADC_GetCalibrationStatus(ADC2) & ADC_GetCalibrationStatus(ADC3)
           & ADC_GetCalibrationStatus(ADC4))
    {
    }

    /*Enable Temp and Vrefint*/
    ADC_EnableTempSensorVrefint(ENABLE);

    /*ADC1 Injected group of conversions end and Analog Watchdog interruptsenabling */
    ADC_ConfigInt(ADC1, ADC_INT_JENDC | ADC_INT_AWD, ENABLE);
    ADC_ConfigInt(ADC3, ADC_INT_JENDC | ADC_INT_AWD, ENABLE);
}

/**
 * @brief  Configures the Tim1 or Tim8.
 */
void TIM_PwmConfig(TIM_Module* TIMx)
{
    TIM_TimeBaseInitType TIMx_TimeBaseStructure;
    OCInitType TIMx_OCInitStructure;
    TIM_BDTRInitType TIMx_BDTRInitStructure;
    NVIC_InitType NVIC_InitStructure;

    uint16_t TimerPeriod = 0;

    TimerPeriod = (SystemCoreClock / 20000) - 1;

    /*Time Base configuration*/
    TIM_DeInit(TIMx);
    TIM_InitTimBaseStruct(&TIMx_TimeBaseStructure);
    TIMx_TimeBaseStructure.Prescaler = 0;
    TIMx_TimeBaseStructure.CntMode   = TIM_CNT_MODE_CENTER_ALIGN2; // 01:/\,irq flag only counter down
    TIMx_TimeBaseStructure.Period    = TimerPeriod;                // PWM_PERIOD;
    TIMx_TimeBaseStructure.ClkDiv    = 0;                          // TIM_CLK_DIV2;
    TIMx_TimeBaseStructure.RepetCnt =  0; // REP_RATE;// Initial condition is REP=0 to set the UPDATE only on the underflow

    TIM_InitTimeBase(TIMx, &TIMx_TimeBaseStructure);
    /*Channel 1, 2,3 in PWM mode */
    TIM_InitOcStruct(&TIMx_OCInitStructure);
    TIMx_OCInitStructure.OcMode = TIM_OCMODE_PWM1; // when '<' is active,when '>' is inactive
    TIMx_OCInitStructure.OutputState  = TIM_OUTPUT_STATE_DISABLE;
    TIMx_OCInitStructure.OutputNState = TIM_OUTPUT_NSTATE_DISABLE;
    TIMx_OCInitStructure.Pulse        = (TimerPeriod >> 1); // dummy value
    TIMx_OCInitStructure.OcPolarity   = TIM_OC_POLARITY_HIGH;
    TIMx_OCInitStructure.OcNPolarity  = TIM_OCN_POLARITY_HIGH;
    TIMx_OCInitStructure.OcIdleState  = TIM_OC_IDLE_STATE_RESET;
    TIMx_OCInitStructure.OcNIdleState = TIM_OC_IDLE_STATE_RESET;
    TIM_InitOc1(TIMx, &TIMx_OCInitStructure);
    TIM_InitOc2(TIMx, &TIMx_OCInitStructure);
    TIM_InitOc3(TIMx, &TIMx_OCInitStructure);
    /*Channel 4 Configuration in OC */
    TIMx_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIMx_OCInitStructure.Pulse       = TimerPeriod - 200;
    TIM_InitOc4(TIMx, &TIMx_OCInitStructure);

    /*Enables the TIM1 Preload on CC1,CC2,CC3,CC4 Register */
    TIM_ConfigOc1Preload(TIMx, TIM_OC_PRE_LOAD_ENABLE);
    TIM_ConfigOc2Preload(TIMx, TIM_OC_PRE_LOAD_ENABLE);
    TIM_ConfigOc3Preload(TIMx, TIM_OC_PRE_LOAD_ENABLE);

    /*Automatic Output enable, Break, dead time and lock configuration*/
    TIMx_BDTRInitStructure.OssrState = TIM_OSSR_STATE_ENABLE;
    TIMx_BDTRInitStructure.OssiState = TIM_OSSI_STATE_ENABLE;
    TIMx_BDTRInitStructure.LockLevel = TIM_LOCK_LEVEL_OFF;//TIM_LOCK_LEVEL_1;
    TIMx_BDTRInitStructure.DeadTime  = 72;                          // DEADTIME;
    TIMx_BDTRInitStructure.Break = TIM_BREAK_IN_ENABLE;             // TIM_BREAK_IN_ENABLE;TIM_BREAK_IN_DISABLE
    TIMx_BDTRInitStructure.BreakPolarity = TIM_BREAK_POLARITY_HIGH; // TIM_BREAK_POLARITY_HIGH;//TIM_BREAK_POLARITY_LOW;
    TIMx_BDTRInitStructure.AutomaticOutput = TIM_AUTO_OUTPUT_ENABLE;//TIM_AUTO_OUTPUT_DISABLE; // TIM_AUTO_OUTPUT_DISABLE;//TIM_AUTO_OUTPUT_DISABLE;
    TIM_ConfigBkdt(TIMx, &TIMx_BDTRInitStructure);
    /*Sel Output Trigger*/
    TIM_SelectOutputTrig(TIMx, TIM_TRGO_SRC_OC4REF); // Master mode select,010:The Update event as trigger output(TRGO)
    /*IT about*/
    TIM_ClrIntPendingBit(TIMx, TIM_INT_BREAK);
    TIM_ConfigInt(TIMx, TIM_INT_BREAK, ENABLE);

    /*Enable the TIMx BRK Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = (TIMx == TIM1) ? TIM1_BRK_IRQn : TIM8_BRK_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*TIMx counter enable*/
    TIM_Enable(TIMx, ENABLE);
    TIM_EnableCtrlPwmOutputs(TIMx, ENABLE);
}

/**
 * @brief  Configures the Tim1 or Tim8 cc output enable.
 */
void TIM_AllPwmOpen(TIM_Module* TIMx)
{
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC1NEN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC2NEN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC3NEN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC1EN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC2EN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC3EN));
}

/**
 * @brief  Configures the Tim1 or Tim8 cc output shut.
 */
void TIM_AllPwmShut(TIM_Module* TIMx)
{
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC1EN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC2EN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC3EN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC1NEN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC2NEN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC3NEN));
}

/**
 * @brief  Configures the Tim1 or Tim8 cc brake
 */
void TIM_Brake(TIM_Module* TIMx)
{
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC1EN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC2EN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC3EN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC1NEN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC2NEN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC3NEN));
}

/**
 * @brief  Configures the Tim1 or Tim8 set cc
 */
void TIM_DutySet(TIM_Module* TIMx, int16_t duty1, int16_t duty2, int16_t duty3)
{
    TIMx->CCDAT1 = duty1;
    TIMx->CCDAT2 = duty2;
    TIMx->CCDAT3 = duty3;
}

/*Irq Adcx Samp value*/
uint16_t ADC_ConvertedValue[4][5];
#define MASK_AD_BITS 0x0FFF // min 7bits

/**
 * @brief  Irq with adc1 and adc2
 */
void ADC1_2_IRQHandler(void)
{
    if (ADC_GetIntStatus(ADC1, ADC_INT_JENDC) == SET)
    {
        ADC_ClearFlag(ADC1, ADC_FLAG_JENDC);
        ADC_ConvertedValue[0][0] = ADC_GetInjectedConversionDat(ADC1, ADC_INJ_CH_1) & MASK_AD_BITS;
        ADC_ConvertedValue[0][1] = ADC_GetInjectedConversionDat(ADC2, ADC_INJ_CH_1) & MASK_AD_BITS;
        ADC_ConvertedValue[0][3] = ADC_GetInjectedConversionDat(ADC1, ADC_INJ_CH_2);
        ADC_ConvertedValue[0][4] = ADC_GetInjectedConversionDat(ADC2, ADC_INJ_CH_2);
    }
    else
    {
        if (ADC_GetIntStatus(ADC1, ADC_INT_AWD) == SET)
            ADC_ClearFlag(ADC1, ADC_FLAG_AWDG);
    }
}

/**
 * @brief  Irq with adc3 and adc4
 */
void ADC3_4_IRQHandler(void)
{
    if (ADC_GetIntStatus(ADC3, ADC_INT_JENDC) == SET)
    {
        ADC_ClearFlag(ADC3, ADC_FLAG_JENDC);
        ADC_ConvertedValue[1][0] = ADC_GetInjectedConversionDat(ADC3, ADC_INJ_CH_1) & MASK_AD_BITS;
        ADC_ConvertedValue[1][1] = ADC_GetInjectedConversionDat(ADC4, ADC_INJ_CH_1) & MASK_AD_BITS;
        ADC_ConvertedValue[1][3] = ADC_GetInjectedConversionDat(ADC3, ADC_INJ_CH_2);
        ADC_ConvertedValue[1][4] = ADC_GetInjectedConversionDat(ADC4, ADC_INJ_CH_2);
    }
    else
    {
        if (ADC_GetIntStatus(ADC3, ADC_INT_AWD) == SET)
            ADC_ClearFlag(ADC3, ADC_FLAG_AWDG);
    }
}

/**
 * @brief  Irq TIM1 break
 */
void TIM1_BRK_IRQHandler()
{
    if (TIM_GetIntStatus(TIM1, TIM_INT_BREAK) != RESET)
    {
        TIM_ClrIntPendingBit(TIM1, TIM_INT_BREAK);
    }

    TIM_AllPwmShut(TIM1);
}

/**
 * @brief  Irq TIM8 break
 */
void TIM8_BRK_IRQHandler()
{
    if (TIM_GetIntStatus(TIM8, TIM_INT_BREAK) != RESET)
    {
        TIM_ClrIntPendingBit(TIM8, TIM_INT_BREAK);
    }

    TIM_AllPwmShut(TIM8);
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

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
 * @file bsp_tsc.c
 * @author Nations
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "bsp_tsc.h"
#include "bsp_gpio.h"

// The following is the corresponding relationship of port, TSC channel, pulse data and key of this demo.
/*  port <-------->  TSC channel    <-------->  key on board

        PA4  <--------> TSC channel 0   <-------->  NC
        PA5  <--------> TSC channel 1   <-------->  NC
        PB14 <--------> TSC channel 2   <-------->  NC
        PB15 <--------> TSC channel 3   <-------->  NC
    
        PD8  <--------> TSC channel 4   <-------->  NC
        PD9  <--------> TSC channel 5   <-------->  NC
        PD11 <--------> TSC channel 6   <-------->  NC
        PD12 <--------> TSC channel 7   <-------->  NC
    
        PC6  <--------> TSC channel 8   <-------->  KEY "K5"
        PC7  <--------> TSC channel 9   <-------->  KEY "K6"
        PC8  <--------> TSC channel 10  <-------->  NC
        PC9  <--------> TSC channel 11  <-------->  NC
    
        PC10 <--------> TSC channel 12  <-------->  NC
        PC11 <--------> TSC channel 13  <-------->  NC
        PC12 <--------> TSC channel 14  <-------->  NC
        PD2  <--------> TSC channel 15  <-------->  NC
    
        PD4  <--------> TSC channel 16  <-------->  NC
        PD5  <--------> TSC channel 17  <-------->  NC
        PD6  <--------> TSC channel 18  <-------->  NC
        PD7  <--------> TSC channel 19  <-------->  NC
    
        PB6  <--------> TSC channel 20  <-------->  NC
        PB7  <--------> TSC channel 21  <-------->  NC
        PB8  <--------> TSC channel 22  <-------->  NC
        PB9  <--------> TSC channel 23  <-------->  NC
    */
    
    //PA9  <------->  TSC_OUT

static uint8_t TSC_Key_ch[KEY_NUM]={8,9};
static uint8_t TSC_Key_Num[KEY_NUM]={5,6};

/******************************************************************/
/***
 * @brief   Init tsc by hardware mode,all channels used.
 * @param:  None
 * @retval: None
 */
void BSP_TSC_HwInit(void)
{
    TSC_InitType Init = {0};
    TSC_AnaoCfg  PadCfg = {0};

    /* Select LSI as TSC clock source and enable */
    TSC_ClockConfig(TSC_CLK_SRC_LSI);

    /* Init gpio port used for TSC */
    tsc_gpio_init();

    /* Init TSC pad option */
    PadCfg.TSC_AnaoptrResisOption   = TSC_PAD_INTERNAL_RES;
    PadCfg.TSC_AnaoptrSpeedOption   = TSC_PAD_SPEED_0;
    TSC_SetAnaoCfg(TSC, &PadCfg);

    /* Config TSC hardwart mode */
    Init.TSC_DetPeriod      = TSC_DET_PERIOD_8_32KHZ;   /* Detect priod= 8/TSC_CLK */
    Init.TSC_FilterCount    = TSC_HW_DET_FILTER_1;      /* Detect filter= 1 sample */
    Init.TSC_LessEnable     = ENABLE;   /* Enable less detect */
    Init.TSC_GreatEnable    = DISABLE;  /* Disable great detect */
    Init.TSC_DetIntEnable   = DISABLE;  /* Disable interrupt */
    TSC_Init(TSC,&Init);
    
    /*Select the internal resistance of each channel as 126K*/
    TSC_ConfigInternalResistor(TSC, TSC_CHN_SEL_ALL, TSC_RESR_CHN_RESIST_125K );
}

/***
 * @brief   Get TSC cnt value of each channel after poweron,and set as threshold
 * @param:  None
 * @retval: None
 */
void BSP_TSC_AutoAdjust(void)
{
    uint32_t i,tCnt;

    /* Config TSC wakeup time,it is controlled by RTC module*/
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR,ENABLE);    /*Enable PWR  peripheral Clock*/
    PWR_BackupAccessEnable(ENABLE);     /* Set bit 8 of PWR_CTRL1.Open PWR DBP.*/
    RCC_ConfigRtcClk(RCC_RTCCLK_SRC_LSI);   /* Set LSI as RTC clock*/
    RCC_EnableRtcClk(ENABLE);           /* Enable RTC clock*/
    RTC_EnableWakeUpTsc(0x3fe);     /* Set TSC wakeup time as 0x2FE*LSI */

    /* Get cnt value of each channel */
    for(i=0;i<KEY_NUM;i++)
    {
        __TSC_CHN_CONFIG(0x1UL << TSC_Key_ch[i]);   /* enable a channel */
        __TSC_HW_ENABLE();              /* enable hw detect */

//        while(__TSC_GET_HW_MODE()==RESET);    /* wait until tsc wakeup and start detect */
        
        while(__TSC_GET_CHN_NUMBER() != TSC_Key_ch[i]); /* make sure current detect channel is correct */

        while((tCnt=__TSC_GET_CHN_CNT())==0);   /* wait until detect done */
        __TSC_HW_DISABLE();     /* disable hw detect */

        TSC_ConfigThreshold(TSC, 0x1UL << TSC_Key_ch[i], tCnt, tCnt/3);  /* set the cnt value as threshold */
    }

    RTC_EnableWakeUpTsc(0x2efa);    /* Set wakeup time as the longest in normal work */
}

/***
 * @brief   Config the interrupt of TSC
 * @param:  None
 * @retval: None
 */
void BSP_TSC_ConfigInt(void)
{
    EXTI_InitType EXTI_InitStruct;

    EXTI_InitStruct.EXTI_Line    = EXTI_LINE21;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;

    EXTI_InitPeripheral(&EXTI_InitStruct);

    NVIC_InitType NVIC_InitStructure;

    /* Configure NVIC as priority group 1. */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Configure the source of interrupt.*/
    NVIC_InitStructure.NVIC_IRQChannel = TSC_IRQn;
    /* Configure preemption priority. */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    /* Configure subpriority. */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    /* Enable interrupt channel. */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    __TSC_INT_ENABLE();
}

/***
 * @brief   Get the key number as show on board.
 * @param:  None
 * @retval: Key number on board,5,6 or 7
 */
uint32_t BSP_TSC_GetKeyNum(void)
{
    uint32_t i,ch;

    ch = __TSC_GET_CHN_NUMBER();

    for(i=0;i<KEY_NUM;i++)
    {
        if(ch == TSC_Key_ch[i])
            return (TSC_Key_Num[i]);
    }

    return 0;
}


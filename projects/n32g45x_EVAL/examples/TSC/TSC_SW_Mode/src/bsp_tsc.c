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
#include "main.h"
#include "bsp_tsc.h"
#include "bsp_gpio.h"
#include "bsp_timer.h"

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


static TSC_ETR_KEY_PARA tsc_etr_key;         // The struct is the about the process of key recognition by ETR.
static bool b_one_cycle_sample_flag = false; // Calculate after completing 10 samples for each channel.
static bool b_channel_switch_delay = false;
static uint16_t *pEtrTimCnt = NULL;

static uint8_t TSC_Key_ch[KEY_NUM]={8,9};
static uint8_t TSC_Key_Num[KEY_NUM]={5,6};

/******************************************************************/
/***
* @brief    Init tsc by software mode,all channel used.
 * @param:  None
 * @retval: None
*/
void BSP_TSC_SW_Init(void)
{
    TSC_AnaoCfg  PadCfg = {0};
    
    /*Initialize the variables of TSC.*/
    memset((void*)&tsc_etr_key, 0, sizeof(TSC_ETR_KEY_PARA));

    /*------------------for test--------------------*/
    uint32_t i;
    for(i=0;i<KEY_NUM;i++)
    {
        tsc_etr_key.MaxPressSum[i]      = 0;
        tsc_etr_key.MaxReleaseSum[i]    = 0;
        tsc_etr_key.MinPressSum[i]      = 0xFFFFFFFF;
        tsc_etr_key.MinReleaseSum[i]    = 0xFFFFFFFF;
    }
    /*-----------------------------------------------*/

    TSC_ClockConfig(TSC_CLK_SRC_LSI);
    
    tsc_gpio_init();

    TSC_EtrTimInit(TIM2);
    pEtrTimCnt = (uint16_t *)(&(TIM2->CNT));

    PadCfg.TSC_AnaoptrResisOption   = TSC_PAD_INTERNAL_RES;
    PadCfg.TSC_AnaoptrSpeedOption   = TSC_PAD_SPEED_0;
    TSC_SetAnaoCfg(TSC, &PadCfg);
    
    /*Select the internal resistance of each channel as 126K*/
    TSC_ConfigInternalResistor(TSC,TSC_CHN_SEL_ALL, TSC_RESR_CHN_RESIST_125K);

    TSC_SW_SwtichChn(TSC, 0x1UL << TSC_Key_ch[0], TIM2, ENABLE);
    b_channel_switch_delay = true;
}
/******************************************************************/

/******************************************************************/
#define THRESHOLD_MIN  400
/**
* @brief  Check if all samples are done every 10ms
*         If they are done,check if there is any change of key status.
*         If key status changed,print out.
* @param: none
* @return: none
*/
void BSP_TSC_KeyCheck(void)
{
    uint32_t i = 0, j = 0, tempSum;

    if (!b_one_cycle_sample_flag)
        return;

    for (i = 0; i < KEY_NUM; i++)
    {
        tempSum = 0;
        for (j = 0; j < SUM_NUM; j++)
        {
            tempSum += tsc_etr_key.PulseSample[i][j];
        }

        if(tsc_etr_key.EtrSumThreshold[i] == 0) /*init the threshold for each channel*/
        {
            if(tempSum < THRESHOLD_MIN)
                tsc_etr_key.EtrSumThreshold[i] = THRESHOLD_MIN;
            else
                tsc_etr_key.EtrSumThreshold[i] = tempSum;
        }

        if ((tempSum <= THRESHOLD_MIN)||((tempSum+200) < tsc_etr_key.EtrSumThreshold[i]))
        {
            tsc_etr_key.key_status |= 1 << i;

            /*------------------for test--------------------*/
            if(tsc_etr_key.MaxPressSum[i] < tempSum)
            tsc_etr_key.MaxPressSum[i] = tempSum;

            if(tsc_etr_key.MinPressSum[i] > tempSum)
            tsc_etr_key.MinPressSum[i] = tempSum;
            /*-----------------------------------------------*/
        }
        else
        {
            tsc_etr_key.key_status &= ~(1 << i);

            /*------------------for test--------------------*/
            if(tsc_etr_key.MaxReleaseSum[i] < tempSum)
            tsc_etr_key.MaxReleaseSum[i] = tempSum;

            if(tsc_etr_key.MinReleaseSum[i] > tempSum)
            tsc_etr_key.MinReleaseSum[i] = tempSum;
            /*-----------------------------------------------*/
        }
    }


    if (tsc_etr_key.key_status != tsc_etr_key.key_status_pre)
    {
        BSP_TSC_KeyPrint(tsc_etr_key.key_status,tsc_etr_key.key_status_pre);
        tsc_etr_key.key_status_pre = tsc_etr_key.key_status;
    }

    b_one_cycle_sample_flag = false;
}
/******************************************************************/

/******************************************************************/
/**
* @brief Print the key press and release infomation.
* @param current_key: key status deteced now
* @param last_key:    key status deteced last time
* @return : none
*/
void BSP_TSC_KeyPrint(uint32_t current_key, uint32_t last_key)
{
    uint32_t i = 0, key_change;

    key_change = current_key^last_key;

    for (i = 0; i < KEY_NUM; i++)
    {
        if (key_change & (1 << i))
        {
            if(current_key & (1<<i))
            {
                printf("K%d pressed,key=0x%03x!\r\n",TSC_Key_Num[i],0x1U<<TSC_Key_ch[i]);
                led_blink = i*2+2;
            }
            else
                printf("K%d released,key=0x%03x!\r\n",TSC_Key_Num[i],0x1U<<TSC_Key_ch[i]);
        }
    }

//    if (current_key == 0)
//    {
//        printf("No valid key!\r\n");
//        return;
//    }
}
/******************************************************************/

/******************************************************************/
/**
* @brief  Do it in TIM3 interrupt process.
*         Calculate the number of pulses for each channel one by one during TIM3 period.
*         The pulses are counted by TIM2
* @param: None
* @return: None
*/
void BSP_TSC_TimIrqCheck(void)
{
    static uint32_t Channel = 0;
    static uint32_t num     = 0;
    uint32_t tempDelta;

    if(b_one_cycle_sample_flag)     /*if the data of last loop is not dealed with user, wait*/
        return;

    /*Keep last value of TIMx->CNT*/
    tsc_etr_key.EtrCntPre = tsc_etr_key.EtrCnt;

    /* Get current value of TIMx->CNT*/
    tsc_etr_key.EtrCnt = *pEtrTimCnt;

    if (b_channel_switch_delay==true)
    {
        b_channel_switch_delay = false;
        return;
    }

    /* Calculate the number of pulses in 100us(between two TIM3 IRQS)*/
    if (tsc_etr_key.EtrCnt >= tsc_etr_key.EtrCntPre)
        tempDelta = tsc_etr_key.EtrCnt - tsc_etr_key.EtrCntPre;
    else
        tempDelta = tsc_etr_key.EtrCnt+0xFFFF-tsc_etr_key.EtrCntPre;

    /*save the pulse for each sample in every Channel*/
    tsc_etr_key.PulseSample[Channel][num] = tempDelta;
    
    if(Channel<(KEY_NUM-1)) /* switch to next channel */
        Channel++;
    else            /* Get sample of last channel */
    {
        Channel = 0;    /* switch to the first channel */
        num++;          /* loop count */
        if(num >= SUM_NUM)       /*Get enough samples for each channel*/
        {
            num = 0;        /* reset the loop count */
            b_one_cycle_sample_flag = true; /*set mark for sample finished*/
        }
    }

    __TSC_SW_CHN_NUM_CONFIG(TSC_Key_ch[Channel]);
    b_channel_switch_delay = true;
}




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
 * @file rtc_hal.c
 * @author Nations
 * @version v1.0.1
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
/* Scheduler includes. */
#include "rtc_hal.h"
#include "bsp_usart.h"

#define RTC_SET_DEFAULT_TIME 0xaf50

SYSTEM_TIME_T   SystemTime;
uint32_t SynchPrediv, AsynchPrediv;

/**
 * @brief  Configures the clock source of RTC.
 * @param   clksrc specifies the valide clock source of RTC
 *   This parameter can be one of the following values:
 *     @arg RTC_CLK_SRC_LSI:              RTC clock source is LSI
 *     @arg RTC_CLK_SRC_LSE:              RTC clock source is LSE,and LSE is oscillator
 *     @arg RTC_CLK_SRC_LSE_BYPASS:       RTC clock source is LSE,and LSE is extennal clock
 *     @arg RTC_CLK_SRC_HSE_DIV128:       RTC clock source is HSE/128
 * @retval None
 */
void RTC_CLKSourceConfig(uint32_t ClkSrc)
{
    /* Enable the PWR clock */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR | RCC_APB1_PERIPH_BKP, ENABLE);
//    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);

    /* Allow access to RTC */
    PWR_BackupAccessEnable(ENABLE);

    /* Disable RTC clock */
    //RCC_EnableRtcClk(DISABLE);

    if (ClkSrc == RTC_CLK_SRC_HSE_DIV128)
    {
        log_info("RTC_ClkSrc is set as HSE/128! \r\n");

        RCC_ConfigHse(RCC_HSE_ENABLE);
        if(RCC_WaitHseStable() != SUCCESS)
        {
            log_error("Set RTC clk error:HSE error\r\n");
            while(1);
        }

        RCC_ConfigRtcClk(RCC_RTCCLK_SRC_HSE_DIV128);

        SynchPrediv  = 0x1E8; // 8M/128 = 62.5KHz
        AsynchPrediv = 0x7F;  // value range: 0-7F
    }
    else if ((ClkSrc == RTC_CLK_SRC_LSE) ||(ClkSrc==RTC_CLK_SRC_LSE_BYPASS))
    {
        log_info("RTC_ClkSrc is set as LSE! \r\n");

        RCC_ConfigLse(ClkSrc & (~RCC_BDCTRL_RTCSEL));
        while (RCC_GetFlagStatus(RCC_FLAG_LSERD) == RESET);
        RCC_ConfigRtcClk(RCC_RTCCLK_SRC_LSE);
        
        SynchPrediv  = 0xFF; // 32.768KHz
        AsynchPrediv = 0x7F; // value range: 0-7F
    }
    else if (ClkSrc == RTC_CLK_SRC_LSI)
    {
        log_info("RTC_ClkSrc is set as LSI! \r\n");

        RCC_EnableLsi(ENABLE);  /* Enable the LSI OSC */
        while (RCC_GetFlagStatus(RCC_FLAG_LSIRD) == RESET);
        RCC_ConfigRtcClk(RCC_RTCCLK_SRC_LSI);

        SynchPrediv  = 0x136; // 39.64928KHz
        AsynchPrediv = 0x7F;  // value range: 0-7F
    }
    else
    {
        log_error("RTC_ClkSrc is error! \r\n");
    }

    /* Enable the RTC Clock */
    RCC_EnableRtcClk(ENABLE);
    RTC_WaitForSynchro();
}


/**
 * @brief  Set system time as default.
 * @param  None
 * @retval None
 */
void RTC_DefaultSystemTime(void)
{
    SystemTime.Date.WeekDay = 3;
    SystemTime.Date.Month   = 20;
    SystemTime.Date.Date    = 11;
    SystemTime.Date.Year    = 19;
    
    SystemTime.Time.H12     = RTC_AM_H12;
    SystemTime.Time.Hours   = 4;
    SystemTime.Time.Minutes = 5;
    SystemTime.Time.Seconds = 1;
}

/**
 * @brief  RTC prescaler config.
 * @param  None
 * @retval None
 */
static void RTC_PrescalerConfig(void)
{
    RTC_InitType RTC_InitStructure;
    
    /* Configure the RTC data register and RTC prescaler */
    RTC_InitStructure.RTC_AsynchPrediv = AsynchPrediv;
    RTC_InitStructure.RTC_SynchPrediv  = SynchPrediv;
    RTC_InitStructure.RTC_HourFormat   = RTC_24HOUR_FORMAT;

    /* Check on RTC init */
    if (RTC_Init(&RTC_InitStructure) == ERROR)
    {
        log_error("RTC Prescaler Config failed \r\n");
    }
}

/**
 * @brief  Send current Date to the terminal.
 * @param  None
 * @retval None
 */
void RTC_DateShow(void)
{
    /* Get the current Date */
    RTC_GetDate(RTC_FORMAT_BIN, &(SystemTime.Date));
    log_debug("//=========== Current Date Display ==============// \r\n");
    log_debug("The current date (WeekDay-Date-Month-Year) is :  %0.2d-%0.2d-%0.2d-%0.2d \r\n",
             SystemTime.Date.WeekDay,
             SystemTime.Date.Date,
             SystemTime.Date.Month,
             SystemTime.Date.Year);
}

/**
 * @brief  Send current time to the terminal.
 * @param  None
 * @retval None
 */
void RTC_TimeShow(void)
{
    /* Get the current Time and Date */
    RTC_GetTime(RTC_FORMAT_BIN, &(SystemTime.Time));
    log_debug("\r\n //============ Current Time Display ===============// \r\n");
    log_debug("\r\n The current time (Hour-Minute-Second) is :  %0.2d:%0.2d:%0.2d \r\n",
             SystemTime.Time.Hours,
             SystemTime.Time.Minutes,
             SystemTime.Time.Seconds);
    
    /* Unfreeze the RTC DAT Register */
    (void)RTC->DATE;
}


///**
// * @brief  Set the date from terminal.
// * @param  None
// * @retval ERRO or SUCCESS
// */
//ErrorStatus RTC_DateRegulate(void)
//{
//    uint8_t tmp;
//    RTC_DateType RTC_Date;
//    
//    log_debug("//==============RTC Date regulate==============//\r\n");
//
//    tmp = RTC_Date.WeekDay;
//    if (tmp == 0xff)
//    {
//        log_debug("Default weekday invalid\r\n");
//        return ERROR;
//    }
//    else
//    {
//        log_debug("Set weekday as %0.2d\r\n", tmp);
//        RTC_Date.WeekDay = tmp;
//    }
//
//    tmp = RTC_Date.Date;
//    if (tmp == 0xff)
//    {
//        log_debug("Default date invalid\r\n");
//        return ERROR;
//    }
//    else
//    {
//        log_debug("Set date as %0.2d\r\n", tmp);
//        RTC_Date.Date = tmp;
//    }
//
//    tmp = RTC_Date.Month;
//    if (tmp == 0xff)
//    {
//        log_debug("Default month invalid\r\n");
//        return ERROR;
//    }
//    else
//    {
//        log_debug("Set month as %0.2d\r\n", tmp);
//        RTC_Date.Month = tmp;
//    }
//
//    tmp = RTC_Date.Year;
//    if (tmp == 0xff)
//    {
//        log_debug("Default year invalid\r\n");
//        return ERROR;
//    }
//    else
//    {
//        log_debug("Set year as %0.2d\r\n", tmp);
//        RTC_Date.Year = tmp;
//    }
//
//    /* Configure the RTC date register */
//    if (RTC_SetDate(RTC_FORMAT_BIN, &RTC_Date) == ERROR)
//    {
//        log_debug(">> !! RTC Set Date failed. !! <<\r\n");
//        return ERROR;
//    }
//    else
//    {
//        log_debug(">> !! RTC Set Date success. !! <<\r\n");
//        RTC_DateShow();
//        return SUCCESS;
//    }
//}
//
///**
// * @brief  Set the time from terminal.
// * @param  None
// * @retval ERRO or SUCCESS
// */
//ErrorStatus RTC_TimeRegulate(void)
//{
//    uint8_t tmp;
//    char str[12];
//    RTC_TimeType RTC_Time;
//    
//    log_debug("//==============RTC Time Regulate=================// \r\n");
//
//    RTC_Time.H12 = RTC_Time.H12;
//
//    log_debug("Please input new time:hour minite second\r\n");
//    scanf("%s",str);
//
//    tmp = RTC_Time.Hours;
//    if (tmp == 0xff)
//    {
//        log_debug("Default hour invalid\r\n");
//        return ERROR;
//    }
//    else
//    {
//        log_debug("Set hour as %0.2d\r\n", tmp);
//        RTC_Time.Hours = tmp_hh;
//    }
//    
//    tmp = RTC_Time.Minutes;
//    if (tmp == 0xff)
//    {
//        log_debug("Default hour invalid\r\n");
//        return ERROR;
//    }
//    else
//    {
//        log_debug("Set minute as %0.2d\r\n", tmp);
//        RTC_Time.Minutes = tmp;
//    }
//
//    tmp = RTC_Time.Seconds;
//    if (tmp == 0xff)
//    {
//        log_debug("Default second invalid\r\n");
//        return ERROR;
//    }
//    else
//    {
//        log_debug("Set second as %0.2d\r\n", tmp);
//        RTC_Time.Seconds = tmp;
//    }
//
//    /* Configure the RTC time register */
//    if (RTC_ConfigTime(RTC_FORMAT_BIN, &RTC_Time) == ERROR)
//    {
//        log_debug(">> !! RTC Set Time failed. !! <<\r\n");
//        return ERROR;
//    }
//    else
//    {
//        log_debug(">> !! RTC Set Time success. !! <<\r\n");
//        RTC_TimeShow();
//        return SUCCESS;
//    }
//}

/**
 * @brief  Set the date from terminal.
 * @param  Cmd:ENABLE or DISABLE
 * @retval None
 */
void EXTI20_RTCWKUP_Configuration(FunctionalState Cmd)
{
    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    EXTI_ClrITPendBit(EXTI_LINE20);
    EXTI_InitStructure.EXTI_Line    = EXTI_LINE20;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);

    /* Enable the RTC WakeUp Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = Cmd;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  RTC wakeup interrupt handler.
 * @param  None
 * @retval None
 */
void RTC_WKUP_IRQHandler(void)
{
    if (RTC_GetITStatus(RTC_INT_WUT) != RESET)
    {
        RTC_GetTime(RTC_FORMAT_BIN, &(SystemTime.Time));
        RTC_ClrIntPendingBit(RTC_INT_WUT);
        EXTI_ClrITPendBit(EXTI_LINE20);
    }
}


/**
 * @brief  Config time .
 * @param  None
 * @retval None
 */
void RTC_TimerWakeUpCfg(void)
{
    /* RTC date time alarm default value*/
    RTC_DefaultSystemTime();

    /* RTC clock source set, default LSE */
//    RTC_CLKSourceConfig(RTC_CLK_SRC_LSI);
    RTC_CLKSourceConfig(RTC_CLK_SRC_LSE);

    RTC_PrescalerConfig();
    log_debug("RTC configured....");

    /* Adjust time by values entered by the user on the hyperterminal */
//    RTC_DateRegulate();
//    RTC_TimeRegulate();
}

/**
 * @brief  Config RTC wakeup parameter.
 * @param  None
 * @retval None
 */
void RTC_TimerWakeUpEnable(void)
{
    /* wake up clock select */
    RTC_ConfigWakeUpClock(RTC_WKUPCLK_RTCCLK_DIV16);
    
    /* wake up timer value */
    //1000 / 32.768K = 30.517578125 us
    //30.517578125 * 16 = 488.28125 us
    //300 * 1000  / 488.28125 = 614-->300ms counter is about 614
    RTC_SetWakeUpCounter(675);  //330ms->51uA
    //RTC_SetWakeUpCounter(818);  //400ms->45uA
    //RTC_SetWakeUpCounter(1024);   //500ms->40uA
    //RTC_SetWakeUpCounter(614 * 3);  //900ms->33.7uA

    //DBG_ConfigPeriph(DBG_STOP, ENABLE);

    EXTI20_RTCWKUP_Configuration(ENABLE);
    
    /* Enable the RTC Wakeup Interrupt */
    RTC_ConfigInt(RTC_INT_WUT, ENABLE);
    RTC_EnableWakeUp(ENABLE);
}


/**
 * @brief  RTC config, check the backup data, and set a certain time if necessary
 * @param  None
 * @retval None
 * - NULL
 */
void RTC_Configuration(void)
{
    RTC_InitType RTC_InitStructure;
    RTC_TimeType RTC_TimeStruct;
    RTC_DateType RTC_DateStruct;

    RTC_CLKSourceConfig(RTC_CLK_SRC_LSE);

    //set RTC clk as 1Hz
    RTC_InitStructure.RTC_AsynchPrediv = 0xFF;
    RTC_InitStructure.RTC_SynchPrediv  = 0x7f;
    RTC_InitStructure.RTC_HourFormat   = RTC_24HOUR_FORMAT;

    RTC_Init(&RTC_InitStructure);

    for (uint32_t i = 0; i < 1000; i++);

    if (RTC_SET_DEFAULT_TIME != BKP_ReadBkpData(BKP_DAT1))
    {
        RTC_TimeStruct.Hours   = 10;
        RTC_TimeStruct.Minutes = 18;
        RTC_TimeStruct.Seconds = 50;
        RTC_ConfigTime(RTC_FORMAT_BIN, &RTC_TimeStruct);

        RTC_DateStruct.Year  = 19;
        RTC_DateStruct.Month = 11;
        RTC_DateStruct.Date  = 18;
        RTC_SetDate(RTC_FORMAT_BIN, &RTC_DateStruct);
        BKP_WriteBkpData(BKP_DAT1, RTC_SET_DEFAULT_TIME);
    }

    rtc_time_get(&SystemTime);
}

/**
 * @brief  Get current date and time
 * @param  SYSTEM_TIME_T *DateTime: specifies a pointer of SYSTEM_TIME_T
 * @retval None
 */
void rtc_time_get(SYSTEM_TIME_T* DateTime)
{
    RTC_GetTime(RTC_FORMAT_BIN, &(DateTime->Time));
    RTC_GetDate(RTC_FORMAT_BIN, &(DateTime->Date));

    log_debug("get system time: 20%02d-%02d-%02d  %02d-%02d-%02d weekday %d\r\n",
            DateTime->Date.Year,
            DateTime->Date.Month,
            DateTime->Date.Date,
            DateTime->Time.Hours,
            DateTime->Time.Minutes,
            DateTime->Time.Seconds,
            DateTime->Date.WeekDay);
}

/**
 * @brief  Set date and time
 * @param  SYSTEM_TIME_T *DateTime: specifies a pointer of SYSTEM_TIME_T
 * @param  format specifies the time format  
 *   This parameter can be one of the following values:
 *     @arg RTC_FORMAT_BIN:  date and time value is BIN format
 *     @arg RTC_FORMAT_BCD:  date and time value is BCD format
 * @retval None
 */
void rtc_time_set(SYSTEM_TIME_T* DateTime, uint32_t format)
{
    SystemTime.Time.Hours   = DateTime->Time.Hours;
    SystemTime.Time.Minutes = DateTime->Time.Minutes;
    SystemTime.Time.Seconds = DateTime->Time.Seconds;
    SystemTime.Time.H12     = DateTime->Time.H12;
    RTC_ConfigTime(format, &(SystemTime.Time));

    SystemTime.Date.Year    = DateTime->Date.Year;
    SystemTime.Date.Month   = DateTime->Date.Month;
    SystemTime.Date.Date    = DateTime->Date.Date;
    SystemTime.Date.WeekDay =  DateTime->Date.WeekDay;
    RTC_SetDate(format, &(SystemTime.Date));

    log_debug("set system time: 20%02d-%02d-%02d  %02d-%02d-%02d\r\n",
            SystemTime.Date.Year,
            SystemTime.Date.Month,
            SystemTime.Date.Date,
            SystemTime.Time.Hours,
            SystemTime.Time.Minutes,
            SystemTime.Time.Seconds);
}

/**
 * @brief  Set date and time
 * @param  SYSTEM_TIME_T *DateTime: specifies a pointer of SYSTEM_TIME_T
 * @param  format specifies the time format  
 *   This parameter can be one of the following values:
 *     @arg RTC_FORMAT_BIN:  date and time value is BIN format
 *     @arg RTC_FORMAT_BCD:  date and time value is BCD format
 * @retval None
 */
//void rtc_time_update(uint8_t in_pos, uint8_t in_val, SYSTEM_TIME_T* DateTime)
//{
//    uint8_t* pdate = (uint8_t*)DateTime;
//    uint8_t m      = (in_pos >> 1);
//
//    if (in_pos % 2 == 0)
//    {
//        pdate[m] = pdate[m] % 10 + in_val * 10; //十位
//    }
//    else if (in_pos % 2 == 1)
//    {
//        pdate[m] = pdate[m] - pdate[m] % 10 + in_val; //个位
//    }
//}


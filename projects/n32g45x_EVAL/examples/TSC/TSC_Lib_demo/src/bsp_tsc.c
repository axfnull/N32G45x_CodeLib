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
#include "bsp_timer.h"
#include "bsp_usart.h"

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

TSC_KEY_INFO TscKey;

//门锁使用的触控通道列表
/* clang-format off */
const TSC_AlgInitThreValue gtsc_list[] = {
    /*  base   thre   rate ofchange   chn */
    {TSC_HOLD_LEV1,     7,        TSC_CHN8},
    {TSC_HOLD_LEV1,     7,        TSC_CHN9}
};
/* clang-format on */


//uint32_t app_touch_get_chn_index(uint32_t userchn)
//{
//    uint32_t i;
//
//    for (i = 0; i <= MAX_TSC_HW_CHN; i++)
//    {
//        if (userchn & 0x00000001)
//        {
//            return i;
//        }
//        userchn >>= 1;
//    }
//    
//    return 0;
//}


/*****************上层应用提供的按键回调处理函数*********
 * @brief 注册按钮型、滑条型、转轮型触控的回调函数
 * @param tsc_touch_type type 产生的触控类型(暂只支持按键型)
 * @param uint32_t event 0:正常触摸事件；1:压下持续时间(2S通知一次),2:校准完成事件
 * @param uint32_t chn 表示触摸通道号；
 * @param uint32_t value 触摸状态：1压下；0松开；或压下事件的持续时间(单位:秒)
 * @return
 * - `TSC_SUCCESS： 表示操作成功
 * - 其它值表示出错
 * 注意:此回调函数将在中断中调用，因此尽量减少回调函数的处理时间。
 ********************************************************/
int32_t tsc_alg_isr_callback(tsc_alg_type type, uint32_t event, uint32_t chn, uint32_t value)
{
//    uint32_t wakeup_type;
    uint8_t cnt = 0;

    if (type == TSC_ALG_BUTTON_TYPE)
    {
        if (event == TSC_PRESS_KEY_NORMAL) {//正常按键操作
            for (cnt = 0; cnt < KEY_NUM; cnt++)
            {
                if (chn == (uint32_t)(gtsc_list[cnt].chn))
                {
                    log_debug("Key [%x] press %d\r\n", (unsigned int)(gtsc_list[cnt].chn), (int)value);
                    if (value)
                    {
                        LED6_ON();
                    }
                    else
                    {
                        LED6_OFF();
                    }
                }
            }
        }
    }
    return TSC_SUCCESS;
}

#ifdef THRE_DATA2PC_DEBUG //输出触控数据到PC分析
/**
 * @brief 触控数据输出到PC的接口，以便于PC端工具观察，设定合理的触控阈值
 * @param uint32_t chn 触控通道
 * @return uint8_t data 该触控通道数据
 */
void tsc_alg_debug_output(uint32_t chn, uint8_t data)
{
    Usart_SendArray(DEBUG_USARTx, (uint8_t *)(&chn), 4);
    Usart_SendByte(DEBUG_USARTx, data);
}
#endif

/**
 * @brief 触控算法库初始化
 * @param  None
 * @retval tsc_ret_code_e
 */
int32_t app_touch_init(void)
{
    int32_t ret;
    uint32_t totals_chn, size;
    TSC_AlgInitTypeDef TSC_AlgStructure = {0};

    //获取总通道数
    totals_chn = sizeof(gtsc_list) / sizeof(gtsc_list[0]);

    //打印触控算法版本
    log_info("tsc version = %s.\r\n", tsc_alg_get_version());

    //判断通道数所需要的SRAM资源是否足够
    size = tsc_alg_need_sramsize(totals_chn);
    if(!size)
    {
        log_error("Get ram size error!\r\n");
        return TSC_SOURCE_NOT_ENOUGH_ERR;
    }
    else if(sizeof(TscKey.sram) < size)
    {
        log_error("Sram is too small,(%d)chn need ram %d,but support size is %d\r\n",(int)totals_chn, (int)size, sizeof(TscKey.sram));
        return TSC_SOURCE_NOT_ENOUGH_ERR;
    }
    else 
    {
        log_info("tsc_alg_need_sramsize ok:size=%d\r\n",(int)size);
    }

    //初始化触控模块及BUF
    TSC_AlgStructure.TIMx               = TIM2;
    TSC_AlgStructure.DMAyChx            = DMA1_CH5;
    TSC_AlgStructure.DMARemapEnable     = 1;
    TSC_AlgStructure.pTScChannelList    = (TSC_AlgInitThreValue*)gtsc_list;
    TSC_AlgStructure.AutoCalibrateTimer = 1000;                //覆盖物的自动校准时间1000ms
    TSC_AlgStructure.ResistDisturbLev   = TSC_RESIST_DIS_LEV0; //抗干扰等级。装配成品后建议加亚克力，并配置为TSC_RESIST_DIS_LEV1
    TSC_AlgStructure.pTscSramAddr       = (uint8_t*)(TscKey.sram);
    TSC_AlgStructure.TscSramSize        = sizeof(TscKey.sram);
#ifdef THRE_TOUCHDATA_DEBUG //监控模式下设定log缓存空间
    TSC_AlgStructure.LogBuf             = (uint16_t*)(TscKey.LogBuf);
    TSC_AlgStructure.LogBufSize         = sizeof(TscKey.LogBuf) / sizeof(TscKey.LogBuf[0]);
#endif
    TSC_AlgStructure.Stop2Data             = (uint8_t *)gTscStop2Data;
    TSC_AlgStructure.Stop2DataSize         = sizeof(gTscStop2Data);
    ret                                 = tsc_alg_init(&TSC_AlgStructure);
    if (ret != TSC_SUCCESS)
    {
        log_error( "tsc_alg_init errcode = %x.\r\n", (unsigned int)ret);
        return ret;
    } else {
        log_info("tsc_alg_init ok.\r\n");
    }

    //启动触控开始工作
    ret = tsc_alg_start();
    if (ret != TSC_SUCCESS)
    {
        log_error( "errcode = %x.\r\n", (unsigned int)ret);
        return ret;
    } 
    else 
    {
        log_info("tsc_alg_start ok.\r\n");
    }

    return ret;
}

/**
 * @brief 触控检测功能初始化
 * @param  None
 * @retval tsc_ret_code_e
 */
int32_t touch_ctrl_init(void)
{
    int32_t ret;
    uint32_t totals_chn;

    memset((void *)(TscKey.KeyHold), 0, sizeof(TscKey.KeyHold));
    TscKey.Status = 0;
    TscKey.WakeupFlag = 0;
    TscKey.WakeupDelay = 0;

    ret = app_touch_init();
    if (ret != TSC_SUCCESS)
    {
        return ret;
    }

    //获取总通道数
    totals_chn = sizeof(gtsc_list) / sizeof(gtsc_list[0]);

    //开始算法分析定时器7
    TouchAnalyzeTimerInit(totals_chn);
    TouchAnalyzeTimerEnable(ENABLE);

    return TSC_SUCCESS;
}

/**
 * @brief  关闭定时器,使能TSC中断
 * @param  None
 * @retval tsc_ret_code_e
 */
int32_t touch_ctrl_powerdown(void)
{
    NVIC_InitType NVIC_InitStructure;

    // SOTP TIM6
    TIM_Enable(TIM6, DISABLE);

    NVIC_InitStructure.NVIC_IRQChannel    = TIM6_IRQn; // TIM7中断
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;   // IRQ通道被禁能
    NVIC_Init(&NVIC_InitStructure);                    //初始化NVIC寄存器

    // STOP TIM7
    TIM_Enable(TIM7, DISABLE);
    NVIC_InitStructure.NVIC_IRQChannel    = TIM7_IRQn; // TIM7中断
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;   // IRQ通道被禁能
    NVIC_Init(&NVIC_InitStructure);                    //初始化NVIC寄存器

    
    return TSC_SUCCESS;
}

/**
 * @brief 唤醒检测
 * @param  None
 * @return 0:正常唤醒；1:干扰唤醒
 */
uint32_t touch_wakeup_check(void)
{
    int32_t ret;
    uint32_t func_ret = 0;
    uint32_t wakeup_src = 0;
    uint32_t totals_chn;

    TscKey.WakeupFlag = 0;
    TscKey.WakeupDelay = 0;
    log_init();
    ret = app_touch_init();
    if (ret != TSC_SUCCESS)
    {
        log_debug( "touch_wakeup_check:app_touch_init failed\r\n");
        return 1;
    }

    //开始计数定时器6
    TIM6_init();

    //获取总通道数
    totals_chn = sizeof(gtsc_list) / sizeof(gtsc_list[0]);

    //开始算法分析定时器7
    TouchAnalyzeTimerInit(totals_chn);
    TouchAnalyzeTimerEnable(ENABLE);

    //初步判断
    ret = tsc_alg_wakeup_disturb_check(&wakeup_src);
    if (ret == TSC_SUCCESS) 
    {
        if (wakeup_src) 
        {
            log_info( "quick check distrub.\r\n");
            func_ret = 1;
        } 
        else 
        {
            //再进一步判断,等待150ms进行判断处理.
            while(TscKey.WakeupDelay <= 150) 
            {
                __NOP();
            }
            
            if (TscKey.WakeupFlag == 0) 
            {
                //异常干扰唤醒
                log_info( "slowly check distrub.\r\n");

                func_ret = 1;
            } 
            else 
            {
                log_info( "slowly no distrub.\r\n");
            }
        }
    } 
    else 
    {
        log_error( "quick check failded.\r\n");
        func_ret = 1;
    }

    TIM_Enable(TIM6, DISABLE);
    TouchAnalyzeTimerEnable(DISABLE);

    //设置触控低功耗唤醒
    tsc_alg_set_powerdown(0);

    return func_ret;
}

/**
 * @brief 正常模式触控功能
 * @param  None
 * @retval None
 */
static void touch_normal(void)
{
    uint32_t i, j;
//    uint8_t key_value;

    for (i = TSC_CHNEN_CHN_SEL0, j = 0; i <= TSC_CHNEN_CHN_SEL23; i <<= 1, j++)
    {
    }
}

/**
 * @brief 触控状态机
 * @param  None
 * @return None
 */
void touch_ctrl(void)
{
    switch (TscKey.Status) {
        case 0://正常模式
            touch_normal();
            break;

        case 1://校准中...
            break;

        default:
            TscKey.Status = 0;
            break;
    }
}

/**
 * @brief 上电后只需调用一次；在反复STOP2唤醒过程中无需再调用(重要)
 * @param void
 * 注意：对使用TSC功能的应用来说，只有上电调用，其他情况下不调用.
 * 注意：对使用TSC功能的应用来说，只有上电调用，其他情况下不调用.
 * 注意：对使用TSC功能的应用来说，只有上电调用，其他情况下不调用.
 */
void Tsc_PoweronInit(void)
{
    uint32_t i;

    TSC_ClockConfig(TSC_CLK_SRC_LSI);   /* Default clock source is LSE */

    for (i = TSC_CHNEN_CHN_SEL0; i <= TSC_CHNEN_CHN_SEL23; i <<= 1)
    {
        TSC_ConfigThreshold(TSC,i,0,0);
        TSC_ConfigInternalResistor(TSC,i,TSC_RESR_CHN_RESIST_1M);
    }

    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_BKP, ENABLE);

    //初始化相关变量
    memset((void*)gTscStop2Data, 0, sizeof(gTscStop2Data));
}


void tsc_wakeup_init(void)
{
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TSC, ENABLE);
 
    // open rcc PWREN BKPEN
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_BKP | RCC_APB1_PERIPH_PWR, ENABLE);
 
    // Set bit 8 of PWR_CTRL // open PWR DBKP
    PWR->CTRL |= PWR_CTRL_DBKP;

    RTC_EnableWakeUpTsc(TSC_ALG_WAKEUP_TIMES);
}


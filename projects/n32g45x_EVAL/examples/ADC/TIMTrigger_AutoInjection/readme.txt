1、功能说明
    1、ADC1规则采样PC2、自动注入采样PC3引脚的模拟电压，在TIM1 CC2事件下触发采样
    2、其中规则转换结果通过DMA_CH1通道读取到变量ADC_RegularConvertedValueTab[32]数组
       注入转换结果通过转换结束中断读取到变量ADC_InjectedConvertedValueTab[32]数组
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0 EVB开发
3、使用说明
    系统配置；
        1、时钟源：
            HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,ADC CLK=144M/16,ADC 1M CLK=HSE/8,DMA CLK=144M,TIM1 CLK=144M
        2、中断：
            ADC1注入转换结果完成中断打开，抢断优先级0，子优先级0
            中断函数中将注入结果读取到ADC_InjectedConvertedValueTab[32]数组，并翻转PC6电平
        3、端口配置：
            PC2选择为模拟功能ADC1转换通道
            PC3选择为模拟功能ADC1转换通道
            PC6选择为通用IO输出
            PA8选择为TIM1 CH1 PWM输出
            PA11选择为TIM1 CH4 PWM输出
        4、DMA：
            DMA1_CH1通道回环模式搬运32个半字的ADC1转换结果到ADC_RegularConvertedValueTab[32]数组
        5、ADC：
            ADC1独立工作模式、TIM1 CC2触发、12位数据右对齐，规则转换通道PC2，自动注入转换通道PC3的模拟电压数据
        6、TIM：
            TIM1开启CH1 CH4输出，CH2用作触发ADC转换        
    使用方法：
        1、编译后打开调试模式，将变量ADC_RegularConvertedValueTab[32],ADC_InjectedConvertedValueTab[32]添加到watch窗口观察
        2、通过改变PC2 PC3引脚的电压，在每个CC2事件发生时转换一次规则和注入通道，变量储存在对应数组中。同时在PA8 PA11可以看到TIM1 CH1和CH4的 PWM 波形
4、注意事项
    当系统采用HSE时钟时（一般HSI也是打开的），RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8)可以配置为HSE或者HSI
    当系统采用HSI时钟时（一般HSE是关闭的），RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8)只能配置为HSI
1、功能说明
    1、ADC1、ADC2、ADC3、ADC4分别采样PC2、PC3、PD13、PD14引脚的模拟电压，各自独立转换
    2、其中ADC1转换结果通过DMA_CH1通道读取到变量ADC1ConvertedValue
       ADC2转换结果通过转换结束中断读取到变量ADC2ConvertedValue
       ADC3转换结果通过DMA_CH5通道读取到变量ADC3ConvertedValue
       ADC4转换结果通过转换结束中断读取到变量ADC4ConvertedValue
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0 EVB开发
3、使用说明
    系统配置；
        1、时钟源：
            HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,ADC CLK=144M/16,ADC 1M CLK=HSE/8,DMA CLK=144M
        2、中断：
            ADC2转换结果完成中断打开，抢断优先级0，子优先级0
            ADC4转换结果完成中断打开，抢断优先级0，子优先级0
        3、端口配置：
            PC2选择为模拟功能ADC1转换通道8
            PC3选择为模拟功能ADC2转换通道9
            PD13选择为模拟功能ADC3转换通道10
            PD14选择为模拟功能ADC4转换通道11
        4、DMA：
            DMA1_CH1通道回环模式搬运一个半字的ADC1转换结果到ADC1ConvertedValue变量
            DMA2_CH5通道回环模式搬运一个半字的ADC3转换结果到ADC3ConvertedValue变量
        5、ADC：
            ADC1独立工作模式、连续转换、软件触发、12位数据右对齐，转换通道8即PC2的模拟电压数据
            ADC2独立工作模式、连续转换、软件触发、12位数据右对齐，转换通道9即PC3的模拟电压数据
            ADC3独立工作模式、连续转换、软件触发、12位数据右对齐，转换通道10即PD13的模拟电压数据
            ADC4独立工作模式、连续转换、软件触发、12位数据右对齐，转换通道11即PD14的模拟电压数据
    使用方法：
        1、编译后打开调试模式，将变量ADC1ConvertedValue,ADC2ConvertedValue,ADC3ConvertedValue,ADC4ConvertedValue添加到watch窗口观察
        2、通过改变PC2 PC3 PD13 PD14引脚的电压，可以看到转换结果变量同步改变
4、注意事项
    当系统采用HSE时钟时（一般HSI也是打开的），RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8)可以配置为HSE或者HSI
    当系统采用HSI时钟时（一般HSE是关闭的），RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8)只能配置为HSI
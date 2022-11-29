1、功能说明
    1、ADC1、ADC2、ADC3、ADC4分别采样PC2、PC3、PD13、PD14引脚的模拟电压，各自独立转换
    2、其中ADC1转换结果通过DMA_CH1通道读取到变量ADC1ConvertedValue
       ADC2转换结果通过转换结束中断读取到变量ADC2ConvertedValue
       ADC3转换结果通过DMA_CH5通道读取到变量ADC3ConvertedValue
       ADC4转换结果通过转换结束中断读取到变量ADC4ConvertedValue
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：        基于N32G4XV-STB V1.0开发
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
            DMA1_CH1通道循环模式搬运一个半字的ADC1转换结果到ADC1ConvertedValue变量
            DMA2_CH5通道循环模式搬运一个半字的ADC3转换结果到ADC3ConvertedValue变量
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


1. Function description
    1. ADC1, ADC2, ADC3 and ADC4 sample the analog voltage of the pins of PC2, PC3, PD13 and PD14 respectively and convert them independently
    2. ADC1 conversion result is read to variable ADC1ConvertedValue through DMA_CH1 channel
       The ADC2 conversion result is read into the variable ADC2ConvertedValue through the conversion end interrupt
       ADC3 conversion results are read to variable ADC3ConvertedValue through DMA_CH5 channel
       The ADC4 conversion result is read into the variable ADC4ConvertedValue through the conversion end interrupt
2. Use environment
    Software development environment: KEIL MDK-ARM V5.26.2.0
    Hardware environment: Developed based on the development board N32G4XV-STB V1.0
3. Instructions for use
    System configuration;
        1. Clock source:
            HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,ADC CLK=144M/16,ADC 1M CLK=HSE/8,DMA CLK=144M
        2. Interrupt:
            ADC2 conversion complete interrupt open, steal priority 0, sub priority 0
            ADC4 conversion complete interrupt open, steal priority 0, sub priority 0
        3. Port Configuration:
            PC2 is selected for analog function, ADC1 conversion channel 8
            PC3 is selected for analog function, ADC2 conversion channel 9
            PD13 is selected for analog function, ADC3 conversion channel 10
            PD14 is selected for analog function, ADC4 conversion channel 11
        4, DMA:
            DMA1_CH1 channel is configuered as circular mode, carries a half-word of ADC1 conversion result to the ADC1ConvertedValue variable
            DMA2_CH5 channel is configuredas circular mode, carries a half-word of ADC3 conversion result to the ADC3ConvertedValue variable
        5, ADC:
            ADC1 configuration: independent mode, continuous conversion, software trigger, 12 bit data right aligned, conversion channel 8 is PC2 analog voltage data
            ADC2 configuration: independent mode, continuous conversion, software trigger, 12 bit data right aligned, conversion channel 9 is PC3 analog voltage data
            ADC3 configuration: independent mode, continuous conversion, software trigger, 12 bit data right aligned, conversion channel 10 is PD13 analog voltage data
            ADC4 configuration: independent mode, continuous conversion, software trigger, 12 bit data right aligned, conversion channel 11 is PD14 analog voltage data
    Instructions:
        1, compiled to open the debug mode, variable ADC1ConvertedValue, ADC2ConvertedValue，ADC3ConvertedValue and ADC4ConvertedValue added to the watch window to observe
        2. By changing the voltage of pin PC2, PC3, PD13, PD14, it can be seen that the conversion result variable changes synchronously
4. Matters needing attention
    When the system uses the HSE clock (HSI is generally enabled), ), RCC_ConfigAdc1mClk (RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8) can be configured as HSE or HSI
    When the system uses the HSI clock(HSE is generally disabled), RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8) can only be configured as HSI
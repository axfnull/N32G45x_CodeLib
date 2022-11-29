1、功能说明
    1、ADC1 ADC2规则通道同步采样，ADC1采样PC2、PC3引脚的模拟电压，ADC2采样PA4、PA5引脚的模拟电压
    2、同步转换的结果通过DMA_CH1传输到ADC_DualConvertedValueTab[16]数组，只传输16次
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0开发
3、使用说明
    系统配置；
        1、时钟源：
            HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,ADC CLK=144M/16,ADC 1M CLK=HSE/8,DMA CLK=144M
        2、端口配置：
            PC2选择为模拟功能ADC1转换通道
            PC3选择为模拟功能ADC1转换通道
            PA4选择为模拟功能ADC2转换通道
            PA5选择为模拟功能ADC2转换通道
        3、DMA：
            DMA1_CH1通道正常模式搬运16个字的ADC1、ADC2同步规则通道转换结果到ADC_DualConvertedValueTab[16]数组
        4、ADC：
            ADC1规则通道同步规则通道模式、连续扫描模式、软件触发、12位数据右对齐，转换通道PC2和PC3的模拟电压数据
            ADC2规则通道同步规则通道模式、连续扫描模式、软件触发、12位数据右对齐，转换通道PA4和PA5的模拟电压数据
    使用方法：
        1、编译后打开调试模式，将变量ADC_DualConvertedValueTab[16]添加到watch窗口观察
        2、全速运行前，在PC2 PC3 PA4 PA5给确定的电压信号，全速运行后，即可转换16次结果到ADC_DualConvertedValueTab[16]数组
4、注意事项
    ADC转换虽然是连续的，但是DMA只存储前16次数据到数组
    当系统采用HSE时钟时（一般HSI也是打开的），RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8)可以配置为HSE或者HSI
    当系统采用HSI时钟时（一般HSE是关闭的），RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8)只能配置为HSI

1. Function description
    1. ADC1 ADC2 regular channel synchronous sampling, ADC1 samples the analog voltage of pin PC2 and PC3, ADC2 samples the analog voltage of pin PA4 and PA5
    2. The result of synchronous conversion is transmitted to ADC_DualConvertedValueTab[16] array through DMA_CH1, only 16 times
2. Use environment
    Software development environment: KEIL MDK-ARM V5.26.2.0
    Hardware environment: Developed based on the development board N32G4XV-STB V1.0
3. Instructions for use
    System configuration;
        1. Clock source:
            HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,ADC CLK=144M/16,ADC 1M CLK=HSE/8,DMA CLK=144M
        2. Port Configuration:
            PC2 is selected as the analog function, ADC1 conversion channel
            PC3 is selected as the analog function, ADC1 conversion channel
            PA4 is selected as the analog function, ADC2 conversion channel
            PA5 is selected as the analog function, ADC2 conversion channel
        3, DMA:
            DMA1_CH1 channel is configuered as normal mode,  carries the 16 word of ADC1 and ADC2 synchronization regular channel conversion results to the ADC_DualConvertedValueTab[16] array
        4, ADC:
            ADC1 configuration: Synchronization regular mode, continuous scan mode, software trigger, 12 bit data right alignment, conversion channels PC2 and PC3 analog voltage data
            ADC2 configuration: Synchronization regular mode, continuous scan mode, software trigger, 12 bit data right alignment, conversion channels PA4 and PA5 analog voltage data
      Instructions:
        1. After compiling, open the debugging mode and add the variable ADC_DualConvertedValueTab [16] to the Watch window for observation
        2. Before running at full speed, PC2, PC3, PA4, PA5 give a certain voltage signal. After running at full speed, the result can be converted 16 times to ADC_DualConvertedValueTab[16] array
4. Matters needing attention
    ADC conversions are continuous, but DMA only stores the first 16 times of data into the array
    When the system uses the HSE clock (HSI is generally enabled), ), RCC_ConfigAdc1mClk (RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8) can be configured as HSE or HSI
    When the system uses the HSI clock(HSE is generally disabled), RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8) can only be configured as HSI
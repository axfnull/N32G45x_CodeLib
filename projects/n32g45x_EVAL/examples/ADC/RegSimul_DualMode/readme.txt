1、功能说明
    1、ADC1 ADC2规则通道同步采样，ADC1采样PC2、PC3引脚的模拟电压，ADC2采样PA4、PA5引脚的模拟电压
    2、同步转换的结果通过DMA_CH1传输到ADC_DualConvertedValueTab[16]数组，只传输16次
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0 EVB开发
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
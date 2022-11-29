1、功能说明
    1、ADC1采样转换PC0 PC1引脚的模拟电压，ADC2采样转换PC2 PC3引脚的模拟电压
    2、采用软件触发一次，采集一次的方式
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0 开发
3、使用说明
    系统配置；
        1、时钟源：
            HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,ADC CLK=144M/16,ADC 1M CLK=HSE/8
        2、端口配置：
            PC0选择为模拟功能ADC1转换通道
            PC1选择为模拟功能ADC1转换通道
            PC2选择为模拟功能ADC2转换通道
            PC3选择为模拟功能ADC2转换通道
        3、ADC：
            ADC1独立工作模式、软件触发转换、12位数据右对齐，转换PC0 PC1的模拟电压数据
            ADC2独立工作模式、软件触发转换、12位数据右对齐，转换PC2 PC3的模拟电压数据
    使用方法：
        1、编译后打开调试模式，将变量ADC1ConvertedValue,ADC2ConvertedValue添加到watch窗口观察
        2、通过改变PC0,PC1,PC2,PC3引脚的电压，可以看到转换结果变量同步改变
4、注意事项
    当系统采用HSE时钟时（一般HSI也是打开的），RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8)可以配置为HSE或者HSI
    当系统采用HSI时钟时（一般HSE是关闭的），RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8)只能配置为HSI


1. Function description
  1. ADC1 samples and converts the analog voltage of PC0 and PC1 pins, ADC2 samples and converts the analog voltage of PC2 and PC3 pins
  2. Use the software to trigger once and collect once
2. Use environment
    Software development environment: KEIL MDK-ARM V5.26.2.0
    Hardware environment: Developed based on the development board N32G4XV-STB V1.0
3. Instructions for use
    System configuration;
        1. Clock source:
            HSE = 8M, PLL = 144M, AHB = 144M, APB1 = 36M, APB2 = 72M, ADC CLK = 144M / 16, ADC 1M CLK = HSE / 8
        2. Port Configuration:
            PC0 is selected for analog function, ADC1 conversion channel 
            PC1 is selected for analog function, ADC1 conversion channel 
            PC2 is selected for analog function, ADC2 conversion channel 
            PC3 is selected for analog function, ADC2 conversion channel 
        3, ADC:
            ADC1 configuration: independent mode, software triggered conversion, 12 bit data right aligned, convert analog voltage data of PC0 and PC1
            ADC2 configuration: independent mode, software triggered conversion, 12 bit data right aligned, convert analog voltage data of PC2 and PC3
    Instructions:
        1, compiled to open the debug mode, variable ADC1ConvertedValue and ADC2ConvertedValue added to the watch window to observe
        2, By changing the voltage of pin PC0, PC1, PC2 and PC3, can see the conversion result variable synchronous change
4. Matters needing attention
    When the system uses the HSE clock (HSI is generally enabled), ), RCC_ConfigAdc1mClk (RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8) can be configured as HSE or HSI
    When the system uses the HSI clock(HSE is generally disabled), RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8) can only be configured as HSI
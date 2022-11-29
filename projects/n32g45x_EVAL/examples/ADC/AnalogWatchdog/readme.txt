1、功能说明
    1、ADC1采样转换PC2引脚的模拟电压，如果超过模拟看门狗定义的阈值范围，则跳入中断程序
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：         基于N32G4XV-STB V1.0 开发
3、使用说明
    系统配置；
        1、时钟源：
            HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,ADC CLK=144M/16,ADC 1M CLK=HSE/8
        2、ADC：
            ADC1独立工作模式、连续转换、软件触发、12位数据右对齐，转换PC2引脚的模拟电压数据
        3、端口配置：
            PC2选择为模拟功能ADC1转换通道8
        4、中断：
            ADC1模拟看门狗中断打开，抢断优先级0，子优先级0
    使用方法：
        1、编译后打开调试模式，将变量gCntAwdg添加到watch窗口观察
        2、改变PC2引脚电压值，当电压值超出模拟看门狗定义的0x300到0xB00范围外，则进入一次中断，变量做累加操作
4、注意事项
    当系统采用HSE时钟时（一般HSI也是打开的），RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8)可以配置为HSE或者HSI
    当系统采用HSI时钟时（一般HSE是关闭的），RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8)只能配置为HSI


1. Function description
  1. ADC1 samples and converts the analog voltage of PC2 pin. If it exceeds the threshold range defined by the analog watchdog, it will jump into the interrupt program
2. Use environment
    Software development environment: KEIL MDK-ARM V5.26.2.0
    Hardware environment: Developed based on the development board N32G4XV-STB V1.0
3. Instructions for use
    System configuration;
        1. Clock source:
            HSE = 8M, PLL = 144M, AHB = 144M, APB1 = 36M, APB2 = 72M, ADC CLK = 144M / 16, ADC CLK = 1M HSE / 8
        2, ADC:
            ADC1 configuration: independent mode, continuous conversion, software trigger, 12 bit data right aligned, conversion analog voltage data of PC2 
        3. Port Configuration:
            PC2 is selected for analog function, ADC1 conversion channel 8
        4. Interruption:
            ADC1 analog watchdog interrupt open, steal priority 0, sub priority 0
    Instructions:
        1. After compiling, open the debugging mode and add the variable gCntAwdg to the Watch window for observation
        2. Change the voltage value of PC2 pin. When the voltage value exceeds the range of 0x300 to 0xB00 defined by the analog watchdog, an interrupt will be entered and the variables will be accumulated
4. Matters needing attention
    When the system uses the HSE clock (HSI is generally enabled), ), RCC_ConfigAdc1mClk (RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8) can be configured as HSE or HSI
    When the system uses the HSI clock(HSE is generally disabled), RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8) can only be configured as HSI
1、功能说明
    1、ADC1采样转换内部温度传感器的模拟电压，并转换为温度值
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：        基于N32G4XV-STB V1.0开发
3、使用说明
    系统配置；
        1、时钟源：
            HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,ADC CLK=144M/16,ADC 1M CLK=HSE/8,DMA CLK=144M
        2、DMA：
            DMA1_CH1通道循环模式搬运一个半字的ADC1转换结果到ADCConvertedValue变量
        3、ADC：
            ADC1独立工作模式、连续转换、软件触发、12位数据右对齐，转换通道16即内部温度传感器的模拟电压数据
        4、端口配置：
            PA9选择为USART1的TX引脚
            PA10选择为USART1的RX引脚
        5、USART：
            USART1 115200波特率、8位数据位、1位停止位、无奇偶校验位、无硬件流控、发送和接收使能
        6、功能函数：
            TempValue = TempCal(ADCConvertedValue)函数将温度ADC原始格式数据转为度的单位的格式
    使用方法：
        1、编译后打开调试模式，将变量ADCConvertedValue,TempValue添加到watch窗口观察
        2、将串口工具连接到PA9引脚，并打开串口接收工具
        3、全速运行，可以看到温度变量的数值在常温下接近25度左右，同时串口工具显示实时芯片内的温度值
4、注意事项
    当系统采用HSE时钟时（一般HSI也是打开的），RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8)可以配置为HSE或者HSI
    当系统采用SI时钟时（一般HSE是关闭的），RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8)只能配置为HSI


1. Function description
    1. ADC1 samples and converts the analog voltage of the internal temperature sensor to the temperature value
2. Use environment
    Software development environment: KEIL MDK-ARM V5.26.2.0
    Hardware environment: Developed based on the evaluation board N32G4XV-STB V1.0
3. Instructions for use
    System configuration;
        1. Clock source:
            HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,ADC CLK=144M/16,ADC 1M CLK=HSE/8,DMA CLK=144M
        2, DMA:
            DMA1_CH1 channel is configuered as circular mode, carries a half-word of ADC1 conversion result to the ADC1ConvertedValue variable
        3, ADC:
            ADC1 configuration: independent mode, continuous conversion, software trigger, 12 bit data right aligned, conversion channel 16is the analog voltage data of the internal temperature sensor
        4. Port Configuration:
            PA9 is the TX pin of USART1
            PA10 is the RX pin for USART1
        5, USART:
            USART1 configuration: 115200 Baud rate, 8 data bits, 1 Stop bit, no parity bit, no hardware flow control, send and receive enabled
        6. Functions:
            The TempValue = TempCal(ADCConvertedValue) function converts temperature ADC raw format data into degrees
    Instructions:
        1, compiled to open the debug mode, variable ADCConvertedValue, TempValue added to the watch window to observe
        2. Connect the serial port tool to the PA9 pin and open the serial port receiver tool
        3. Running at full speed, it can be seen that the value of the temperature variable is close to 25 degrees at room temperature, and the serial port tool displays the real-time temperature value in the chip
4. Matters needing attention
    When the system uses the HSE clock (HSI is generally enabled), ), RCC_ConfigAdc1mClk (RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8) can be configured as HSE or HSI
    When the system uses the HSI clock(HSE is generally disabled), RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8) can only be configured as HSI
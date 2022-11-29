1、功能说明
    1、TIM1 CH4触发ADC1  ADC2同步注入采样OPA1 OPA2的电压数据，TIM8 CH4触发ADC3  ADC4同步注入采样OPA3 OPA4的电压数据，并且TIM1 TIM8输出受COMP1和COMP7刹车控制
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：        基于N32G4XV-STB V1.0 开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,ADC CLK=144M/10,ADC 1M CLK=HSE/8,OPA CLK=36M,COMP CLK=36M,TIM1 CLK=144M,TIM8 CLK=144M
        2、中断：
                    TIM1刹车中断打开，抢断优先级0，子优先级0
                    TIM8刹车中断打开，抢断优先级0，子优先级0
        3、端口配置：
                    PA5选择为模拟功能OPA1 VP
                    PB0选择为模拟功能OPA2 VP
                    PC9选择为模拟功能OPA3 VP
                    PC5选择为模拟功能OPA4 VP
                    PA6选择为模拟功能OPA1 OUT
                    PB1选择为模拟功能OPA2 OUT
                    PB11选择为模拟功能OPA3 OUT
                    PB12选择为模拟功能OPA4 OUT
                    PA1选择为模拟功能COMP1 INP
                    PA11选择为模拟功能COMP1 OUT
                    PC1选择为模拟功能COMP7 INP
                    PC2选择为模拟功能COMP7 OUT
                    PA8选择为TIM1 CH1输出
                    PA9选择为TIM1 CH2输出
                    PA10选择为TIM1 CH3输出
                    PB13选择为TIM1 CH1N输出
                    PB14选择为TIM1 CH2N输出
                    PB15选择为TIM1 CH3N输出
                    PC6选择为TIM8 CH1输出
                    PC7选择为TIM8 CH2输出
                    PC8选择为TIM8 CH3输出
                    PA15选择为TIM8 CH1N输出
                    PC12选择为TIM8 CH2N输出
                    PD2选择为TIM8 CH3N输出
                    PA0选择为ADC1_CH1通道输入
                    PC0选择为ADC12_CH6通道输入
        4、OPA：
                    OPA1 OPA2 OPA3 OPA4电压跟随器功能，VM无引脚
        5、ADC：
                    ADC1同步注入模式、扫描转换、TIM1 CC4触发、12位数据右对齐，注入转换OPA1输出电压和内部通道16的模拟电压数据
                    ADC2同步注入模式、扫描转换、TIM1 CC4触发、12位数据右对齐，注入转换OPA2输出电压和内部通道18的模拟电压数据
                    ADC1同步注入模式、扫描转换、TIM8 CC4触发、12位数据右对齐，注入转换OPA3输出电压和内部通道18的模拟电压数据
                    ADC2同步注入模式、扫描转换、TIM8 CC4触发、12位数据右对齐，注入转换OPA4输出电压和内部通道18的模拟电压数据
        6、COMP：
                    COMP1 INP选择PA1，INM选择1.65V，输出触发TIM1和TIM8刹车
                    COMP7 INP选择PC1，INM选择1.65V，输出触发TIM1和TIM8刹车
        7、TIM：
                    TIM1 6路互补打开，刹车使能且打开中断，CH4触发输出
                    TIM8 6路互补打开，刹车使能且打开中断，CH4触发输出
    使用方法：
        1、编译后打开调试模式，用示波器或者逻辑分析仪观察TIM1 TIM8的输出波形和ADC读取的采集数据
        2、改变COMP1 COMP7的输出状态可以刹车TIM1或TIM8，改变OPA1 OPA2 OPA3 OPA4的输入电压可以改变ADC读取的数据
4、注意事项
    无


1. Function description
    1. TIM1 CH4 triggers ADC1 and ADC2 to synchronously inject and sample the voltage data of OPA1 and OPA2, TIM8 CH4 triggers ADC3 and ADC4 to synchronously inject and sample the voltage data of OPA3 and OPA4, and the output of TIM1 and TIM8 is controlled by the brakes of COMP1 and COMP7
2. Use environment
    Software development environment: KEIL MDK-ARM V5.26.2.0
    Hardware environment: Developed based on the development board N32G4XV-STB V1.0
3. Instructions for use
    System Configuration;
        1. Clock source:
                    HSE=8M, PLL=144M, AHB=144M, APB1=36M, APB2=72M, ADC CLK=144M/10, ADC 1M CLK=HSE/8, OPA CLK=36M, COMP CLK=36M, TIM1 CLK=144M, TIM8 CLK=144M
        2. Interrupt:
                    TIM1 brake interrupt is turned on, steal priority is 0, subpriority 0
                    TIM8 brake interrupt is turned on, steal priority is 0, subpriority is 0
        3. Port configuration:
                    PA5 is selected as the analog function OPA1 VP
                    PB0 is selected as the analog function OPA2 VP
                    PC9 is selected as the analog function OPA3 VP
                    PC5 is selected as the analog function OPA4 VP
                    PA6 is selected as the analog function OPA1 OUT
                    PB1 is selected as the analog function OPA2 OUT
                    PB11 is selected as the analog function OPA3 OUT
                    PB12 is selected as the analog function OPA4 OUT
                    PA1 is selected as the analog function COMP1 INP
                    PA11 is selected as the analog function COMP1 OUT
                    PC1 is selected as the analog function COMP7 INP
                    PC2 is selected as the analog function COMP7 OUT
                    PA8 is selected as TIM1 CH1 output
                    PA9 is selected as TIM1 CH2 output
                    PA10 is selected as TIM1 CH3 output
                    PB13 is selected as TIM1 CH1N output
                    PB14 is selected as TIM1 CH2N output
                    PB15 is selected as TIM1 CH3N output
                    PC6 is selected as TIM8 CH1 output
                    PC7 is selected as TIM8 CH2 output
                    PC8 is selected as TIM8 CH3 output
                    PA15 is selected as TIM8 CH1N output
                    PC12 is selected as TIM8 CH2N output
                    PD2 is selected as TIM8 CH3N output
                    PA0 is selected as ADC1_CH1 channel input
                    PC0 is selected as ADC12_CH6 channel input
        4. OPA:
                    OPA1 OPA2 OPA3 OPA4 voltage follower function, VM has no pin
        5. ADC:
                    ADC1 synchronous injection mode, scan conversion, TIM1 CC4 trigger, 12-bit data right-aligned, injection conversion OPA1 output voltage and internal channel 16 analog voltage data
                    ADC2 synchronous injection mode, scan conversion, TIM1 CC4 trigger, 12-bit data right-aligned, injection conversion OPA2 output voltage and internal channel 18 analog voltage data
                    ADC1 synchronous injection mode, scan conversion, TIM8 CC4 trigger, 12-bit data right-aligned, injection conversion OPA3 output voltage and internal channel 18 analog voltage data
                    ADC2 synchronous injection mode, scan conversion, TIM8 CC4 trigger, 12-bit data right-aligned, injection conversion OPA4 output voltage and internal channel 18 analog voltage data
        6. COMP:
                    COMP1 INP selects PA1, INM selects 1.65V, output triggers TIM1 and TIM8 brakes
                    COMP7 INP selects PC1, INM selects 1.65V, output triggers TIM1 and TIM8 brakes
        7. TIM:
                    TIM1 6 channels complementary open, brake is enabled and interrupt is opened, CH4 triggers the output
                    TIM8 6-way complementary open, brake enabled and open interrupt, CH4 trigger output
    Instructions:
        1. After compiling, turn on the debug mode, use an oscilloscope or logic analyzer to observe the output waveform of TIM1 TIM8 and the collected data read by ADC
        2. Changing the output state of COMP1 COMP7 can brake TIM1 or TIM8, and changing the input voltage of OPA1 OPA2 OPA3 OPA4 can change the data read by ADC
4. Matters needing attention
    without
1、功能说明
    1、TIM1 CH4触发ADC1  ADC2同步注入采样OPA1 OPA2的电压数据，TIM8 CH4触发ADC3  ADC4同步注入采样OPA3 OPA4的电压数据，并且TIM1 TIM8输出受COMP1和COMP7刹车控制
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0 EVB开发
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
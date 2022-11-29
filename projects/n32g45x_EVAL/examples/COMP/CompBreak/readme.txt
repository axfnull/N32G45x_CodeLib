1、功能说明
    1、COMP1的输出刹车TIM1 TIM8的互补信号，COMP OUT变低后恢复TIM1 TIM8波形
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0 EVB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,COMP CLK=36M,TIM1 CLK=144M,TIM8 CLK=144M
        2、端口配置：
                    PB10选择为模拟功能COMP INP
                    PA4选择为模拟功能COMP INM
                    PA0选择为模拟功能COMP OUT
                    PE2选择为IO输出
                    PE3选择为IO输出
                    PA8选择为TIM1 CH1输出
                    PA9选择为TIM1 CH2输出
                    PA10选择为TIM1 CH3输出
                    PB13选择为TIM1 CH1N输出
                    PB14选择为TIM1 CH2N输出
                    PB15选择为TIM1 CH3N输出
                    PA11选择为TIM1 CH4输出
                    PB12选择为TIM1 breakin输入
                    PC6选择为TIM8 CH1输出
                    PC7选择为TIM8 CH2输出
                    PC8选择为TIM8 CH3输出
                    PA7选择为TIM8 CH1N输出
                    PB0选择为TIM8 CH2N输出
                    PB1选择为TIM8 CH3N输出
        3、TIM：
                    TIM1开启CH1 CH2 CH3 CH1N CH2N CH3N输出,COMP作为刹车输入
                    TIM8开启CH1 CH2 CH3 CH1N CH2N CH3N输出,COMP作为刹车输入
        4、COMP：
                    COMP1输出触发TIM1 TIM8刹车，无输出时恢复TIM1 TIM8输出
    使用方法：
        1、编译后打开调试模式，将PE2连接到PB10，PE3连接到PA4，利用示波器或者逻辑分析仪观察TIM1 TIM8输出波形
        2、当软件输出PE2电平大于PE3时，TIM波形消失，相反时，波形正常输出
4、注意事项
    无
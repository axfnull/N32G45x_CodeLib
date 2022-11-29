1、功能说明
    1、TIM1 CH3 CH3N互补信号每6个周期改变一次占空比
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0 EVB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,TIM1 CLK=144M,DMA CLK=144M
        2、端口配置：
                    PA10选择为TIM1 CH3输出
                    PB15选择为TIM1 CH3N输出
        3、TIM：
                    TIM1 CH3 CH3N互补输出，每6个周期触发一次DMA传输
        4、DMA：
                    DMA1_CH5通道回环模式搬运3个半字SRC_Buffer[3]变量到TIM1 CC3寄存器
    使用方法：
        1、编译后打开调试模式，用示波器或者逻辑分析仪观察TIM1 CH3 CH3N的波形
        2、TIM1的6个周期改变一次CH3 CH3N的占空比，循环改变
4、注意事项
    无
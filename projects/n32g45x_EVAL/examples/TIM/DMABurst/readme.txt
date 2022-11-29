1、功能说明
    1、TIM1 一个周期后同时改变周期和占空比
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0 EVB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,TIM1 CLK=144M,DMA CLK=144M
        2、端口配置：
                    PA8选择为TIM1 CH1输出
        3、TIM：
                    TIM1 CH1 输出，周期触发DMA burst传输，加载AR,REPCNT,CCDAT1寄存器，改变占空比和周期和重复计数器
        4、DMA：
                    DMA1_CH5通道正常模式搬运3个半字SRC_Buffer[3]变量到TIM1 DMA寄存器
    使用方法：
        1、编译后打开调试模式，用示波器或者逻辑分析仪观察TIM1 CH1的波形
        2、TIM1的第一个周期结束后，后面的波形为DMA搬运的改变周期和占空比的波形
4、注意事项
    无
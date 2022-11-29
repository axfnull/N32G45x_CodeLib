1、功能说明
    1、TIM3 CH1 CH2 CH3 CH4 达到CC值后，输出ACTIVE电平
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0 EVB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,TIM3 CLK=72M
        2、端口配置：
                    PA6选择为TIM3 CH1输出
                    PA7选择为TIM3 CH2输出
                    PB0选择为TIM3 CH3输出
                    PB1选择为TIM3 CH4输出
                    PC6选择为IO 输出
        3、TIM：
                    TIM3 CH2 上升沿捕获中断打开
    使用方法：
        1、编译后打开调试模式，用示波器或者逻辑分析仪观察TIM2 CH1 CH2 CH3 CH4的波形
        2、定时器运行到CC1 CC2 CC3 CC4之后，对应通道的输出变为Active
4、注意事项
    无
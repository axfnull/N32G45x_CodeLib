1、功能说明
    1、TIM2 CH1 CH2 CH3 CH4 达到CC值后，对应拉低PC6 PC7 PC8 PC9的IO电平
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0 EVB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,TIM2 CLK=72M
        2、中断：
                    TIM2 比较中断打开，抢断优先级0，子优先级1
        3、端口配置：
                    PC6选择为IO 输出
                    PC7选择为IO 输出
                    PC8选择为IO 输出
                    PC9选择为IO 输出
        4、TIM：
                    TIM2 配置好CH1 CH2 CH3 CH4的比较值，并打开比较中断
    使用方法：
        1、编译后打开调试模式，用示波器或者逻辑分析仪观察PC6 PC7 PC8 PC9的波形
        2、定时器运进入CC1 CC2 CC3 CC4中断后,对应拉低PC6 PC7 PC8 PC9的IO
4、注意事项
    无
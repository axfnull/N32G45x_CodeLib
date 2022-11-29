1、功能说明
    1、TIM4 CH2上升沿触发CH1输出一个单脉冲
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0 EVB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,TIM4 CLK=72M
        2、端口配置：
                    PB6选择为TIM4的CH1输出
                    PB7选择为TIM4的CH2输入
                    PA3选择为IO输出
        3、TIM：
                    TIM4 配置CH2上升沿触发CH1输出一个单脉冲
    使用方法：
        1、编译后打开调试模式，PA3连接PB7，用示波器或者逻辑分析仪观察TIM4 的CH1 的波形
        2、程序发送PA3的上升沿，TIM4 CH1输出一个单脉冲
4、注意事项
    无
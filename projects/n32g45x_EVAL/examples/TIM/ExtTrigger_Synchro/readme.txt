1、功能说明
    1、TIM1 CH2门控CH1和TIM3 TIM4
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0 EVB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,TIM1 CLK=144M,TIM3 CLK=72M,TIM4 CLK=72M
        2、端口配置：
                    PA8选择为TIM1 CH1输出
                    PA9选择为TIM1 CH2输入
                    PA6选择为TIM3 CH1输出
                    PB6选择为TIM4 CH1输出
        3、TIM：
                    TIM1 CH2 门控CH1，门控TIM3,TIM4
    使用方法：
        1、编译后打开调试模式，用示波器或者逻辑分析仪观察TIM1 CH1,TIM3 CH1,TIM4 CH1的波形
        2、TIM1 CH2高电平定时器开始计数，低电平停止
4、注意事项
    无
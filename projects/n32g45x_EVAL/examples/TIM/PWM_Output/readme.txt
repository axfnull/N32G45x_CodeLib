1、功能说明
    1、TIM3 CH1 CH2 CH3 CH4输出频率相同占空比不同的PWM
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0 EVB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,TIM3 CLK=72M
        2、端口配置：
                    PA6选择为TIM3的CH1输出
                    PA7选择为TIM3的CH2输出
                    PB0选择为TIM3的CH3输出
                    PB1选择为TIM3的CH4输出
        3、TIM：
                    TIM3 CH1 CH2 CH3 CH4周期相等，占空比不等
    使用方法：
        1、编译后打开调试模式，用示波器或者逻辑分析仪观察TIM3 CH1、CH2、CH3、CH4的波形
        2、程序运行后，产生4路周期相等占空比不同的PWM信号
4、注意事项
    无
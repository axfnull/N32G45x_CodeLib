1、功能说明
    1、TIM3 CH2捕获引脚通过CH1下降沿和CH2上升沿计算占空比和频率
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0 EVB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,TIM3 CLK=72M
        2、中断：
                    TIM3 CC2比较中断打开，抢断优先级0，子优先级0
        3、端口配置：
                    PA7选择为TIM3的CH2输入
                    PA0选择为IO输出
        4、TIM：
                    TIM3 CH1下降沿捕获CH2信号，CH2上升沿捕获CH2信号
    使用方法：
        1、编译后打开调试模式，连接PA0和PA7，将Frequency、DutyCycle添加到watch窗口
        2、程序运行后，PA0发送的脉冲数据可以被捕获到周期和频率到变量
4、注意事项
    无
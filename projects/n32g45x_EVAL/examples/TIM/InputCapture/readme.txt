1、功能说明
    1、TIM3 CH2上升沿计算频率
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0 EVB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,TIM3 CLK=72M
        2、中断：
                    TIM3 CH2上升沿中断打开，抢断优先级0，子优先级1
        3、端口配置：
                    PA7选择为TIM3 CH2输入
                    PA3选择为IO 输出
        4、TIM：
                    TIM3 CH2 上升沿捕获中断打开,捕获的频率范围最小为((TIM3 CLK)/0xffff)Hz,最大频率为((TIM3 CLK)/1)Hz
    使用方法：
        1、编译后打开调试模式，连接PA3与PA7，将变量TIM3Freq添加到watch窗口
        2、程序控制PA3电平翻转后，查看TIM3Freq计算的频率值
4、注意事项
    无
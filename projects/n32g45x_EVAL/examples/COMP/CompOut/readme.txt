1、功能说明
    1、COMP1的输出PB1受输入INP PB10和INM PA4的影响
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0 EVB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,COMP CLK=36M
        2、端口配置：
                    PB10选择为模拟功能COMP INP
                    PA4选择为模拟功能COMP INM
                    PB1选择为模拟功能COMP OUT
                    PE2选择为IO输出
                    PE3选择为IO输出
        3、COMP：
                    COMP1输入PB10，PA4，输出PB1
    使用方法：
        1、编译后打开调试模式，将PE2连接到PB10，PE3连接到PA4，利用示波器或者逻辑分析仪观察PB1输出波形
        2、当软件输出PE2电平大于PE3时，PB1输出高电平，相反时，输出低电平
4、注意事项
    无
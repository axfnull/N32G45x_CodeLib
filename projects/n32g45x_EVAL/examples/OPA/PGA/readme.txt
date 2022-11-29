1、功能说明
    1、OPA PGA模式，放大输入电压2倍
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0 EVB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,OPA CLK=36M
        3、端口配置：
                    PA1选择为模拟功能OPA1 VP
                    PA7选择为模拟功能OPA2 VP
                    PC9选择为模拟功能OPA3 VP
                    PC3选择为模拟功能OPA4 VP
                    PA6选择为模拟功能OPA1 OUT
                    PB1选择为模拟功能OPA2 OUT
                    PB11选择为模拟功能OPA3 OUT
                    PB12选择为模拟功能OPA4 OUT
        4、OPA：
                    OPA1 OPA2 OPA3 OPA4 PGA功能，倍数2倍
    使用方法：
        1、编译后打开调试模式，用示波器观察OPA1 OPA2 OPA3 OPA4的输入和输出
        2、OPA输出=2*OPA输入
4、注意事项
    无
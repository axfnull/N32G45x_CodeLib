1、功能说明
    1、软件触发DAC CH1输出4094的电平信号，噪声幅度为1
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G4XV-STB V1.0开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,DAC CLK=36M
        2、端口配置：
                    PA4选择为模拟功能DAC CH1 OUT
        3、DAC：
                    DAC选择软件触发，开启CH1通道幅值为4094，开启噪声模式，噪声幅值为1
        4、主循环：
                    一直触发DAC CH1输出
    使用方法：
        1、编译后打开调试模式，利用示波器观察PA4输出波形
        2、全速运行时，可看到PA4输出大约3.3V电平
4、注意事项
    无



1. Function description
    1. The software triggers the DAC CH1 to output the level signal of 4094, and the noise amplitude is 1
2. Use environment
    Software development environment: KEIL MDK-ARM V5.26.2.0
    Hardware environment: Developed based on the development board N32G4XV-STB V1.0
3. Instructions for use
    System configuration;
        1. Clock source:
            HSE = 8M, PLL = 144M, AHB = 144M, APB1 = 36M, APB2 = 72M, DAC CLK = 36M
        2. Port Configuration:
            PA4 is selected as analog function, DAC CH1 OUT
        3, DAC:
            DAC selects software trigger to open CH1 channel with amplitude of 4094 and noise mode with amplitude of 1
        4. Main loop:
            Always trigger DAC CH1 output
    Instructions:
        1. After compiling, open the debugging mode and observe the OUTPUT waveform of PA4 with oscilloscope
        2. When running at full speed, you can see PA4 output about 3.3V level
4. Matters needing attention
    None

1、功能说明
    1、对备份寄存器进行写入操作


2、使用环境

    软件开发环境：KEIL MDK-ARM V5.25

    硬件环境：
        1、基于评估板N32G45XVL-STB V1.1开发
        2、MCU：N32G457VEL7


3、使用说明
    
    系统配置：
        1、时钟源：HSE+PLL
        2、时钟频率：144



    使用方法：
        在KEIL下编译后烧录到评估板，通过调试器进入调试模式。查看BKP窗口，全速执行后，42个备份寄存器都
              写入了对应的值。比如BKP_DAT1 = 0x3210,BKP_DAT1 = 0x326A


4、注意事项
    无


1. Function description
    1. Write to the backup register

2. Use environment
    Software development environment:
        KEIL MDK-ARM V5.25
    Hardware environment:
        1. Developed based on evaluation board N32G45XVL-STB V1.1
        2. MCU: N32G457VEL7

3. Instructions for use
    System configuration:
    1. Clock source: HSE+PLL
    2. Clock frequency: 144

    Instructions:
        After compiling in KEIL, burn to the evaluation board and enter debugging mode through the debugger. Check the BKP window, and after full speed execution, the corresponding values are written to all 42 backup registers. For example,BKP_DAT1 = 0x3210 and BKP_DAT1 = 0x326A

4. Matters needing attention
    None

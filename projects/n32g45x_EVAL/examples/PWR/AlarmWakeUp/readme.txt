1、功能说明
    1、通过RTC闹钟来唤醒STOP2。
    2、串口打印进入退出状态

2、使用环境

    软件开发环境：KEIL MDK-ARM V5.25

    硬件环境：
        1、基于评估板N32G45XVL-STB V1.1开发
        2、MCU：N32G457VEL7


3、使用说明
    
    系统配置；
        1、RTC时钟源：LSI
        2、低功耗模式：STOP2
        3、串口配置：
            - 串口为USART1（TX：PA9  RX：PA10）:
            - 数据位：8
            - 停止位：1
            - 奇偶校验：无
            - 波特率： 115200 


    使用方法：
        在KEIL下编译后烧录到评估板，通电，过了一会串口打印start low power，说明进入低功耗模式了。
        当串口打印输出Exit low power，说明此时闹钟唤醒STOP2了。


4、注意事项
    无
    
    
1. Function description
    1. Use the RTC alarm to wake up STOP2.
    2. Serial port printing enter and exit state

2. Use environment
    Software development environment: KEIL MDK-ARM V5.25

    Hardware environment:
        1. Developed based on the evaluation board N32G45XVL-STB V1.1
	2. MCU: N32G457VEL7

3. Instructions for use  
    System Configuration;
        1. RTC clock source: LSI
        2. Low power consumption mode: STOP2
        3. Serial port configuration:
            -The serial port is USART1 (TX: PA9 RX: PA10):
            -Data bits: 8
            -Stop bit: 1
            -Parity check: none
            -Baud rate: 115200

     Instructions:
        After compiling under KEIL, burn it to the evaluation board, turn on the power, and print start low power on the serial port after a while, indicating that it has entered the low power mode.
        When the serial port prints Exit low power, it means that the alarm wake up to STOP2.

4. Matters needing attention
    None
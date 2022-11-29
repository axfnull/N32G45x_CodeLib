1、功能说明
    1、SLEEP模式的进入和退出。

2、使用环境

    软件开发环境：KEIL MDK-ARM V5.25

    硬件环境：
        1、基于评估板N32G45XVL-STB V1.1开发
        2、MCU：N32G457VEL7

3、使用说明
    
    系统配置：
        1、时钟源：HSE+PLL
        2、时钟频率：144MHz
        3、唤醒源：PA0
        4、指示灯：PB5

    使用方法：
        在KEIL下编译后烧录到评估板，上电后，PB5控制的指示灯灭。每隔一会按下唤醒按键PA0，
            PB5控制的指示灯电平都会翻转一次。


4、注意事项
    无


1. Function description
    1. Enter and exit SLEEP mode.

2. Use environment

    Software development environment: KEIL MDK-ARM V5.25

    Hardware environment:
        1. Developed based on the evaluation board N32G45XVL-STB V1.1
        2. MCU: N32G457VEL7

3. Instructions for use
    
    System Configuration:
        1. Clock source: HSE+PLL
        2. Clock frequency: 144MHz
        3. Wake-up source: PA0
        4. Indicator light: PB5

     Instructions:
        After compiling under KEIL and burning to the evaluation board, after power on, the indicator light controlled by PB5 is off. Press the wake-up button PA0 every other time,
        The level of the indicator light controlled by PB5 will flip once.

4. Matters needing attention
    None


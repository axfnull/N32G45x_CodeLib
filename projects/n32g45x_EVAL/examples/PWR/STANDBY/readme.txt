1、功能说明
    1、STANDBY模式的进入和退出。

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
               

    使用方法：
        在KEIL下编译后烧录到评估板，接上电流表，上电后发现电流由mA降到了uA级别，
            按下唤醒按键(PA0),电流由回到了mA级别，过了一会又降到uA。


4、注意事项
    在评估功耗的时候，要注意去掉LED去测


1. Function description
    1. Enter and exit STANDBY mode.

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

    Instructions:
        After compiling under KEIL, burn it to the evaluation board, connect the ammeter, and find that the current has dropped from mA to uA after power on.
        Press the wake-up button (PA0), the current returns to mA level, and after a while, it drops to uA.

4. Matters needing attention
    When evaluating power consumption, pay attention to removing the LED


1、功能说明
    1、PVD配置电压产生对应的中断。


2、使用环境

    软件开发环境：KEIL MDK-ARM V5.25

    硬件环境：
        1、基于评估板N32G45XVL-STB V1.1开发
        2、MCU：N32G457VEL7


3、使用说明
    
    系统配置：
        1、时钟源：HSE+PLL
        2、时钟频率：144MHz



    使用方法：
        在KEIL下编译后烧录到评估板，通过仿真器进入调试模式，在PVD_IRQHandler()中设置断点。
                全速运行。然后再把电压调到PVD设置电压附近，此时程序会停在断点处。


4、注意事项
    MSB位可以进行扩展，具体请参考用户手册MSB位使用说明


1. Function description
    1. The PVD configuration voltage generates a corresponding interrupt.


2. Use environment

     Software development environment: KEIL MDK-ARM V5.25

     Hardware environment:
        1. Developed based on the evaluation board N32G45XVL-STB V1.1
        2. MCU: N32G457VEL7

3. Instructions for use
    
     System Configuration:
        1. Clock source: HSE+PLL
        2. Clock frequency: 144MHz

     Instructions:
        After compiling under KEIL, burn it to the evaluation board, enter the debug mode through the emulator, and set a breakpoint in PVD_IRQHandler().
        Run at full speed. Then adjust the voltage to near the PVD setting voltage, and the program will stop at the breakpoint at this time.

4. Matters needing attention
    The MSB bit can be extended, please refer to the user manual MSB bit usage instructions for details


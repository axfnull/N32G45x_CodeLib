1、功能说明

    /* 简单描述工程功能 */
        这个例程配置并演示CAN1和CAN2在正常工作模式下相互收发CAN报文


2、使用环境

    /* 硬件环境：工程对应的开发硬件平台 */
        开发板：N32G45XVL-STBV1.1
        CAN收发器：CAN1和CAN2都外接了CAN收发器


3、使用说明
    
    /* 描述相关模块配置方法；例如:时钟，I/O等 */
        SystemClock：144MHz
        GPIO：USART（TX - PA9，RX - PA10），CAN1（RX - PD8，TX - PD9），CAN2（RX - PB12，TX - PB13）
        CAN1/2：波特率500K，正常工作模式

    /* 描述Demo的测试步骤和现象 */
        1.编译后下载程序复位运行；
        2.查看串口打印信息，当查看到CAN1/2收发都正常的打印信息后表面程序运行正常；


4、注意事项

1. Function description

     /* Briefly describe the engineering function */
         This example configures and demonstrates that CAN1 and CAN2 send and receive CAN messages to and from each other in normal working mode


2. Use environment

     /* Hardware environment: the development hardware platform corresponding to the project */
         Development board: N32G45XVL-STBV1.1
         CAN transceiver: CAN1 and CAN2 are connected with an external CAN transceiver


3. Instructions for use
    
     /* Describe the configuration method of related modules; for example: clock, I/O, etc. */
         SystemClock: 144MHz
         GPIO: USART (TX-PA9, RX-PA10), CAN1 (RX-PD8, TX-PD9), CAN2 (RX-PB12, TX-PB13)
         CAN1/2: baud rate 500K, normal working mode

     /* Describe the test steps and phenomena of Demo */
         1. After compiling, download the program to reset and run;
         2. Check the serial port print information, and the surface program is running normally after 
             viewing the print information that CAN1/2 send and receive normally;


4. Matters needing attention
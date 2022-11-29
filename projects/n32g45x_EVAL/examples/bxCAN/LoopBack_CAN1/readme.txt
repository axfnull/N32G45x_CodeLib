1、功能说明

    /* 简单描述工程功能 */
        这个例程配置并演示CAN1在环回测试模式下收发CAN报文情况


2、使用环境

    /* 硬件环境：工程对应的开发硬件平台 */
        开发板：N32G45XVL-STBV1.1


3、使用说明
    
    /* 描述相关模块配置方法；例如:时钟，I/O等 */
        SystemClock：144MHz
        USART：TX - PA9，RX - PA10，波特率115200
        CAN1：RX - PD0，TX - PD1，波特率500K，环回测试模式

    /* 描述Demo的测试步骤和现象 */
        1.编译后下载程序复位运行；
        2.查看串口打印信息，CAN1把发送的报文当做接收的报文并保存，当查看到打印信息为接收通过时程序运行正常；


4、注意事项

1. Function description

     /* Briefly describe the engineering function */
         This example configures and demonstrates CAN1 sending and receiving CAN messages in loopback test mode


2. Use environment

     /* Hardware environment: the development hardware platform corresponding to the project */
         Development board: N32G45XVL-STBV1.1


3. Instructions for use
    
     /* Describe the configuration method of related modules; for example: clock, I/O, etc. */
         SystemClock: 144MHz
         USART: TX-PA9, RX-PA10, baud rate 115200
         CAN1: RX-PD0, TX-PD1, baud rate 500K, loopback test mode

     /* Describe the test steps and phenomena of Demo */
         1. After compiling, download the program to reset and run;
         2. View the serial port print information, CAN1 treats the sent message as a received message and saves it. 
            When the print information is viewed as the received message, the program runs normally;


4. Matters needing attention
1. 功能说明
    此例程展示在 RT_Thread 系统创建 CAN 设备

2. 使用环境
    硬件环境：工程对应的开发硬件平台 
    开发板：   N32G457QEL_EVB V1.1

3. 使用说明
    描述相关模块配置方法；例如:时钟，I/O等 
         1. SystemClock：144MHz
         2. GPIO：LED：D6--PB10
                        CAN：RX--PD8、TX--PD9
                        日志（USART1）：TX--PA9  RX--PA10，波特率：115200

    描述Demo的测试步骤和现象 
         1. 编译后下载程序复位运行
         2. 本例程在main()里面创建两个线程，test0 线程和 test1 线程，test0 线程用于控制 D6 250ms闪烁，在test1线程测试CAN设备，用CAN卡接上RX和TX，
            在test1线程发送CAN数据，可用CAN卡在上位机上查看发送的数据，发送完以后，由于是环回模式，发送完以后会接收到刚发送的数据，判断接收数据是否正常，用串口打印测试结果

4. 注意事项
    无

1. Function description
    This routine shows how to create a CAN device on the RT_Thread system

2. Use environment
    Hardware environment: development hardware platform corresponding to the project 
    Development board:      N32G457QEL_EVB V1.1

3. Instructions for use
    Describe the configuration method of related modules; for example: clock, I/O, etc. 
        1. SystemClock: 144MHz
        2. GPIO: LED:D6--PB10
                     CAN: RX--PD8、TX--PD9
                     Log(USART1): TX--PA9 RX--PA10 Baud rate: 115200

    Describe the test steps and phenomena of Demo 
        1. After compiling, download the program to reset and run;
        2. This routine creates two threads in main(), test0 thread and test1 thread, test0 thread is used to control D6 250ms flashing, test1 thread is used to test the CAN device, 
            RX and TX are connected with the CAN card, and CAN data is sent in the test1 thread. CAN card can be used in the upper computer to check the data sent, after sending, 
            Because the loopback mode is used, the data just sent is received after the data is sent. Check whether the received data is normal and print the test result using the serial port

4. Matters needing attention
    None.
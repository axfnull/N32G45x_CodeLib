1. 功能说明
    此例程展示在 RT_Thread 系统创建 SPI 设备，并读、写、擦除操作 W25Q128

2. 使用环境
    硬件环境：工程对应的开发硬件平台 
    开发板：   N32G457QEL_EVB V1.1

3. 使用说明
    描述相关模块配置方法；例如:时钟，I/O等 
         1. SystemClock：144MHz
         2. GPIO：SPI1: NSS--PA4、SCK--PA5、MISO--PA6、MOSI--PA7
                        日志：TX--PA9  RX--PA10，波特率：115200

    描述Demo的测试步骤和现象 
         1. 编译后下载程序复位运行
         2. 本例程在main()里面创建两个线程，test0 线程和 test1 线程，test0 线程用于控制 D6 250ms闪烁，test1线程读、写、擦除操作W25Q128，测试结果通过串口打印

4. 注意事项
    要将J29、J30、J31、J32用跳线帽连接起来

1. Function description
    This routine shows how to create an SPI device on the RT_Thread system and read, write, and erase W25Q128

2. Use environment
    Hardware environment: development hardware platform corresponding to the project 
    Development board:      N32G457QEL_EVB V1.1

3. Instructions for use
    Describe the configuration method of related modules; for example: clock, I/O, etc. 
        1. SystemClock: 144MHz
        2. GPIO: SPI1: NSS--PA4、SCK--PA5、MISO--PA6、MOSI--PA7
                     Log: TX--PA9 RX--PA10 Baud rate: 115200

    Describe the test steps and phenomena of Demo 
        1. After compiling, download the program to reset and run;
        2. This routine creates two threads in main(), test0 thread and test1 thread, test0 thread is used to control D6 250ms flashing, test1 thread read, write, erase operation W25Q128, test results printed through the serial port

4. Matters needing attention
    J29, J30, J31 and J32 should be connected with jumper caps
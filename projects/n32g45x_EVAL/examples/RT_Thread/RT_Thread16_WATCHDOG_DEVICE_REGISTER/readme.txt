1. 功能说明
    此例程展示在 RT_Thread 系统创建 WATCHDOG 设备

2. 使用环境
    硬件环境：工程对应的开发硬件平台 
    开发板：   N32G457QE_EVB V1.0

3. 使用说明
    描述相关模块配置方法；例如:时钟，I/O等 
         1. SystemClock：144MHz
         2. GPIO：LED:D6--PB10
                        日志（USART1）：TX--PA9  RX--PA10，波特率：115200

    描述Demo的测试步骤和现象 
         1. 编译后下载程序复位运行
         2. 本例程在main()里面创建两个线程，test0 线程和 test1 线程，test0 线程用于控制 D6 500ms闪烁，在test1线程中测试 IWDG 设备，查找 IWDG 设备，设置超时时间，启动 IWDG，在空闲时喂狗

4. 注意事项
    无

1. Function description
    This routine shows how to create a WATCHDOG device on the RT_Thread system

2. Use environment
    Hardware environment: development hardware platform corresponding to the project 
    Development board:      N32G457QE_EVB V1.0

3. Instructions for use
    Describe the configuration method of related modules; for example: clock, I/O, etc. 
        1. SystemClock: 144MHz
        2. GPIO: LED:D6--PB10
                     Log(USART1): TX--PA9 RX--PA10 Baud rate: 115200

    Describe the test steps and phenomena of Demo 
        1. After compiling, download the program to reset and run;
        2. This routine creates two threads in main(), test0 thread and test1 thread, test0 thread is used to control D6 500ms flashing, test1 thread is used to test the IWDG device, 
            find the IWDG device, set the timeout, start the IWDG, and feed the dog when idle

4. Matters needing attention
    None.
1. 功能说明
    此例程展示在 RT_Thread Finsh 组件

2. 使用环境
    硬件环境：工程对应的开发硬件平台 
    开发板：   N32G457QEL_EVB V1.1

3. 使用说明
    描述相关模块配置方法；例如:时钟，I/O等 
         1. SystemClock：144MHz
         2. GPIO：LED：D6--PB10
                        日志（USART1）：TX--PA9  RX--PA10，波特率：115200

    描述Demo的测试步骤和现象 
         1. 编译后下载程序复位运行
         2. 本例程中创建一个led0线程，在led0线程中D6 500ms周期闪烁
         3. 通过串口工具可以看到打印 finsh>，默认C-style模式，可以通过输入msh()切换到msh模式，再输入help，打印信息如下：
             msh >help
             RT-Thread shell commands:
             version           - show RT-Thread version information
             list_thread      - list thread
             list_sem          - list semaphore in system
             list_mutex      - list mutex in system
             list_timer        - list timer in system
             list_device      - list device in system
             exit                - return to RT-Thread shell mode.
             help               - RT-Thread shell help.
             ps                  - List threads in the system.
             free               - Show the memory usage in the system.

4. 注意事项
    通过串口发送命令时需要勾选发送新行，进入msh模式后，输入exti返回到C-style模式

1. Function description
    This routine is shown in the RT_Thread Finsh component

2. Use environment
    Hardware environment: development hardware platform corresponding to the project 
    Development board:      N32G457QEL_EVB V1.1

3. Instructions for use
    Describe the configuration method of related modules; for example: clock, I/O, etc. 
        1. SystemClock: 144MHz
        2. GPIO: LED: D6--PB10
                     Log(USART1): TX--PA9 RX--PA10 Baud rate: 115200

    Describe the test steps and phenomena of Demo 
        1. After compiling, download the program to reset and run;
        2. This routine creates a led0 thread, in the led0 thread D6 500ms cycle flashing
        3. Print finsh> through the serial tool, the default C-style mode. You can switch to the msh mode by entering msh() and then entering help. The following information is displayed:
             msh >help
             RT-Thread shell commands:
             version           - show RT-Thread version information
             list_thread      - list thread
             list_sem          - list semaphore in system
             list_mutex      - list mutex in system
             list_timer        - list timer in system
             list_device      - list device in system
             exit                - return to RT-Thread shell mode.
             help               - RT-Thread shell help.
             ps                  - List threads in the system.
             free               - Show the memory usage in the system.

4. Matters needing attention
    When sending a command by serial port, you need to select the sending new row. After entering the msh mode, enter exti to return to C-style mode.
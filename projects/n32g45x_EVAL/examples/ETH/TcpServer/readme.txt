1、功能说明

    1、此例程为TCP网络测试例程，用开发板当做简单的TCP服务器。

2、使用环境

    /* 软件开发环境：当前工程使用的软件工具名称及版本号 */
        IDE工具：KEIL MDK-ARM 5.26.2.0

        /* 硬件环境：工程对应的开发硬件平台 */
        开发板：N32G457QEL_EVB V1.2

3、使用说明

    /* 描述相关模块配置方法；例如:时钟，I/O等 */
        SystemClock：100MHz


    /* 描述Demo的测试步骤和现象 */
    1.串口模块连接PA9、PA10.
    2.连接网线。
    3.打开USR-TCP232-Test软件，连接串口。
    4.开发板上电打印出TCPserver的IP地址和端口号。
    5.使用USR-TCP232-Test软件在右侧选择TCPClient.服务器IP地址和端口号填写串口打印的IP地址和端口号。
    6.发送信息，开发板接收到信息后将会返回信息，并在软件中打印出来。

4、注意事项
    1.先插入网线，再将开发板复位运行。
    2.电脑主机和开发板在同一网段。
    3. N32G457QE_EVB V1.0 开发板使用 ETH 需要将跳线 J16、J17连接起来。
    
    
1. Function description
    1. This routine is a TCP network test routine, using the development board as a simple TCP server.

2. Use environment
    /* Software development environment: name and version of the software tool used in the current project */
        IDE tool: KEIL MDK-ARM 5.26.2.0
    /* Hardware environment: the corresponding development hardware platform */
        Development board: N32G457QEL_EVB V1.2

3. Instructions for use
    /* Describe the related module configuration method; For example: clock, I/O, etc. */
        SystemClock: 100 MHZ
    /* Describes the test steps and symptoms of Demo */
	1. Connect the serial port module to PA9 and PA10.
	2. Connect the network cable.
	3. Run the USR-TCP232-Test software and connect the serial port.
	4. Power on the development board and print the IP address and port number of the TCPserver.
	5. Use USR-TCP232-Test to select TCPClient on the right. Server IP address and Port Number Enter the IP address and port number printed on the serial port.
	6. Send information. After receiving the information, the development board will return the information and print it in the software.

4. Matters needing attention
	1. Insert the network cable and reset the development board.
	2. The PC host and development board are on the same network segment.
	3. N32G457QE_EVB V1.0 development board uses ETH to connect jumpers J16 and J17.
	
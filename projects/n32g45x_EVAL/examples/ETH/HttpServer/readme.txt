1、功能说明

    1、此例程为http网络测试例程，用开发板当做简单的网页服务器，通过浏览器访问可现实简单网页

2、使用环境

    /* 软件开发环境：当前工程使用的软件工具名称及版本号 */
        IDE工具：KEIL MDK-ARM 5.26.2.0

        /* 硬件环境：工程对应的开发硬件平台 */
        开发板：N32G457QEL_EVB V1.2

3、使用说明

    /* 描述相关模块配置方法；例如:时钟，I/O等 */
        SystemClock：100MHz


    /* 描述Demo的测试步骤和现象 */
    1.开发板PA9,PA10作为串口输出，串口工具设置波特率为115200
    2.将电脑主机和开发板连接在同一网段内。
        2.编译后下载程序复位运行。
        3.串口工具将会打印信息，包括开发板被分配到的IP地址。
    4.在电脑主机打开浏览器，直接输入ip地址。
    5.浏览器将会显示简单的网页信息。

4、注意事项
    1.先插入网线，再将开发板复位运行。
    2. N32G457QE_EVB V1.0 开发板使用 ETH 需要将跳线 J16、J17连接起来。
    
    
1. Function description
    1. this routine for HTTP network test routine, with the development board as a simple web server, through the browser to access the reality of a simple web page

2. Use environment
    /* Software development environment: name and version of the software tool used in the current project */
        IDE tool: KEIL MDK-ARM 5.26.2.0
    /* Hardware environment: the corresponding development hardware platform */
        Development board: N32G457QEL_EVB V1.2

3. Instructions for use
    /* Describe the related module configuration method; For example: clock, I/O, etc. */
        SystemClock: 100 MHZ
    /* Describes the test steps and symptoms of Demo */
	1. Development board PA9,PA10 as the serial port output, serial port tool set baud rate to 115200
	2. Connect the PC and development board to the same network segment.
	2. After compiling, the downloaded program is reset and running.
	3. The serial port tool will print information, including the IP address assigned to the development board.
	4. Open the browser on the PC and enter the IP address.
	5. The browser will display simple web page information.

4. Matters needing attention
	1. Insert the network cable and reset the development board.
	2. N32G457QE_EVB V1.0 development board uses ETH to connect jumpers J16 and J17.
	
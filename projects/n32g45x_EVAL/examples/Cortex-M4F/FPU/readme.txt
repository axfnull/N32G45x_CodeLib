1、功能说明

    /* 简单描述工程功能 */
        这个例程配置并演示FPU浮点运算单元


2、使用环境

    /* 硬件环境：工程对应的开发硬件平台 */
        开发板：N32G45XVL-STBV1.1


3、使用说明
    
    /* 描述相关模块配置方法；例如:时钟，I/O等 */
        SystemClock：144MHz
        USART：TX - PA9，RX - PA10，波特率115200

    /* 描述Demo的测试步骤和现象 */
        1.编译后下载程序复位运行；
        2.采用Julia集对FPU进行测试，查看打印信息，对比打开或关闭FPU单元时的计算时间；


4、注意事项



1. Function description
	/* A brief description of the engineering function */
	This routine configures and demonstrates the FPU floating point arithmetic unit

2. Use environment
	/* Hardware environment: the corresponding development hardware platform */
	Development board: N32G45XVL-STBV1.1

3. Instructions for use
	/* Describe the related module configuration method; For example: clock, I/O, etc. */
	SystemClock: 144 MHZ
	USART: TX-PA9, RX-PA10, baud rate 115200

	/* Describes the test steps and symptoms of Demo */
	1. Reset and run the downloaded program after compilation;
	2. Use the Julia set to test the FPU, view the printed information, and compare the calculation time of opening or closing the FPU unit.

4. Matters needing attention
1、功能说明

    /* 简单描述工程功能 */
        这个例程配置并演示使用GPIO使能和禁用中断,通过TIM定时器中断中打印信息观察中断的使能和禁用


2、使用环境

    /* 硬件环境：工程对应的开发硬件平台 */
        开发板：N32G45XVL-STBV1.1
        

3、使用说明
    
    /* 描述相关模块配置方法；例如:时钟，I/O等 */
        SystemClock：144MHz
        USART：TX - PA9，波特率115200
        GPIO：PA0为输入上拉模式
        TIM：预分频系数 - （SystemClock/1200-1），周期 - （1200-1），开启定时器中断

    /* 描述Demo的测试步骤和现象 */
        1.编译后下载程序复位运行；
        2.查看串口打印信息，定时器中断信息每1S打印一次，按下WKUP按键打印停止，再次按下则继续打印，表明程序运行正常；


4、注意事项


1. Function description
	/* A brief description of the engineering function */
	This routine configures and demonstrates the use of GPIO enable and disable interrupts, 
	and observes the enabnle and disable of interrupts by printing information in TIM timer interrupts.

2. Use environment
	/* Hardware environment: the corresponding development hardware platform */
	Development board: N32G45XVL-STBV1.1
        
3. Instructions for use    
	/* Describe the related module configuration method; For example: clock, I/O, etc. */
	SystemClock: 144 MHZ
	USART: TX-PA9, baud rate 115200
	GPIO: PA0 is in input pull-up mode
	TIM: Pre-dividing frequency coefficient - (SystemClock/1200-1), period - (1200-1), start timer interrupt

	/* Describes the test steps and symptoms of Demo */
	1. Reset and run the downloaded program after compilation;
	2. Check the serial port printing information. The timer interrupt information is printed every 1S. Press the WKUP button to stop printing, and press it again to continue printing, indicating that the program is running normally.

4. Matters needing attention
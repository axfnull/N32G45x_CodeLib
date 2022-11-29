1、功能说明
    1、WWDG复位功能。


2、使用环境

    软件开发环境：KEIL MDK-ARM V5.25

    硬件环境：
        1、基于评估板N32G45XRL-STB V1.1开发
        2、MCU：N32G457REL7


3、使用说明
    
    系统配置；
        1、WWDG时钟源：PCLK1
        2、窗口值：42.77ms < n <58.25ms
        3、指示灯：PB5
        4、串口配置：
                        - 串口为USART1（TX：PA9  RX：PA10）:
                        - 数据位：8
                        - 停止位：1
                        - 奇偶校验：无
                        - 波特率： 115200               



    使用方法：
        1、在KEIL下编译后烧录到评估板，上电后，PB5控制的指示灯不停的闪烁。说明窗口值刷新正常，代码正常运行。
        2、当把Delay(50)函数参数改成50以上，整个系统将一直处于复位状态。


4、注意事项
     无


1. Function description
    1. WWDG reset function.
	
2. Use environment
    Software development environment: KEIL MDK-ARM V5.25.
	
    Hardware environment: 
		1.Developed based on the evaluation board N32G45XRL-STB V1.1
		2.MCU：N32G457REL7
	
3. Instructions for use
    System Configuration;
        1. WWDG clock source: PCLK1

		2. window value: 42.77ms < N <58.25ms

		3. Indicator light: PB5

		4. Serial port configuration:

			- Serial port: USART1 (TX: PA9 RX: PA10) :

			- Data bit: 8

			- Stop bit: 1

			- Parity check: None

			- Baud rate: 115200

    Instructions:
        1. After compiling under KEIL, it is burned to the evaluation board. After powering on, the indicator light controlled by PB5 keeps flashing. The window value is refreshed properly, and the code is running properly.
		2. When the Delay(50) function parameter is changed to more than 50, the whole system will always be reset.
		
4. Matters needing attention
    None.
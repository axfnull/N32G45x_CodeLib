1、功能说明
    1、IWDG复位功能。


2、使用环境

    软件开发环境：KEIL MDK-ARM V5.25

    硬件环境：
        1、基于评估板N32G45XV-STB V1.1开发
        2、MCU：N32G457VEL7


3、使用说明
    
    系统配置；
        1、IWDG时钟源：LSI/32
        2、超时时间值：250ms
        3、指示灯：PB5
        4、串口配置：
                            - 串口为USART1（TX：PA9  RX：PA10）:
                            - 数据位：8
                            - 停止位：1
                            - 奇偶校验：无
                            - 波特率： 115200               



    使用方法：
        在KEIL下编译后烧录到评估板，上电后，PB5控制的指示灯不停的闪烁。说明IWDG正常喂狗，代码正常运行。
             当把Delay(249)函数参数改成250以上，整个系统将一直处于复位状态。串口会不断打印reset by IWDG。


4、注意事项
     

1. Function description
    1. IWDG reset function.
	
2. Use environment
    Software development environment: KEIL MDK-ARM V5.25.0.0
	
    Hardware environment: 
		1.Developed based on the evaluation board N32G45XV-STB V1.1
		2.MCU：N32G457VEL7
	
3. Instructions for use
    System Configuration;
        1. IWDG clock source: LSI/32

		2. Timeout value: 250ms

		3. Indicator light: PB5

		4. Serial port configuration:

			- Serial port: USART1 (TX: PA9 RX: PA10) :

			- Data bit: 8

			- Stop bit: 1

			- Parity check: None

			- Baud rate: 115200

    Instructions:
        After compiling in KEIL, it was burned to the evaluation board. After powering on,the indicator controled by PB5 pin kept flashing. The IWDG feeds the dog properly and the code runs properly.
		When the Delay(249) function parameter is changed to more than 250, the entire system will always be reset. The serial port continuously prints reset by IWDG.
		
4. Matters needing attention
    None.
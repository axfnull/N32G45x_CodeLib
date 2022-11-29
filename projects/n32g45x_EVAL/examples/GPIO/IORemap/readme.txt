1、功能说明

    1、此例程展示读取端口高低电平，控制 LED（D6、D17）闪烁
    2、控制 JTAG 和 SWD 接口配置成普通 IO 口进行电平翻转


2、使用环境

    /* 硬件环境：工程对应的开发硬件平台 */
    开发板：N32G457QE_EVB V1.0

3、使用说明
    
    /* 描述相关模块配置方法；例如:时钟，I/O等 */
    SystemClock：144MHz
    GPIO：PA0 选择作为读取电平端口，PB10、PB15 控制 LED(D6、D17) 闪烁


    /* 描述Demo的测试步骤和现象 */
    1.编译后下载程序复位运行；
    2.将 PA0 接入 3.3V D17 亮，D6 灭；PA0 断开，D17 灭，D6 亮，用示波器看 PA13、PA14、PA15、PB3、PB4 IO 电平翻转；


4、注意事项
    JTAG 和 SWD 接口配置成普通 IO 后，需要将 BOOT0 管脚接到 3.3V，通过 BOOT 重新下载


1. Function description
    1. this example shows reading port high and low level, control LED (D6, D17) flashing.
	2. Control JTAG and SWD interfaces to be configured as common IO interfaces for level reversal.
	
2. Use environment
    Hardware environment: 
		1.Developed based on the evaluation board N32G457QE_EVB V1.0
	
3. Instructions for use
    /* Describe the related module configuration method; For example: clock, I/O, etc. */
	1. SystemClock: 144 MHZ
	2. GPIO: PA0 is selected as the reading level port, PB10, PB15 control LED(D6, D17) flashing.
	
	/* Describes the test steps and symptoms of Demo */
	1. Reset and run the downloaded program after compilation;
	2. Connect PA0 to 3.3V ,then you can see D17 on and D6 off. PA0 is off, D17 is off, D6 is on, and the IO level of PA13, PA14, PA15, PB3, PB4 is flipped with oscilloscope.
		
4. Matters needing attention
    After the JTAG and SWD interfaces are configured as common I/O, connect the BOOT0 pin to 3.3V and download it through BOOT.
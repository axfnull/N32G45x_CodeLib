1、功能说明

    此例程展示了通过SDIO主机接口与TF卡之间数据读写功能。
             
    首先初始化SDIO主机，再进行TF卡的初始化，
    
    向TF卡写入数据，用DMA2_CH4将Buffer_Block_Tx数据传输到SDIO主机，SDIO主机再将数据写入TF卡，
    在DMA完成写入操作后要等待SDIO主机与TF卡的执行完成，获取到正确的返回状态表示写入完成。
    
    从TF卡读出数据，SDIO主机从TF卡中读出数据到数据寄存器，用DMA2_CH4将SDIO主机接收到的数据
    存到Buffer_Block_Rx中。等待DMA执行完成，等待SDIO读操作执行完成。
    
    比较Buffer_Block_Tx和Buffer_Block_Rx中的数据一致性，输出对比结果到串口

2、使用环境

    软件开发环境：
        IDE工具：KEIL MDK-ARM 5.26.2.0
    
    硬件环境：
        开发板 N32G457-EVB V1.1


3、使用说明
    
    1、SystemClock：144MHz
    2、DMA通道：DMA2_CH4
    3、SDIO 配置：
            D0   -->   PC8          50MHz，AF_PP
            D1   -->   PC9          50MHz，AF_PP
            D2   -->   PC10         50MHz，AF_PP
            D3   -->   PC11         50MHz，AF_PP
            CLK   -->  PC12         50MHz，AF_PP
            CMD   -->  PD2          50MHz，AF_PP

            分频系数：178    （SDIOCLK = HCLK, SDIO_CK = HCLK/(2 + 分频系数)）
            上升沿有效
            禁用旁路
            禁用时钟保持
            总线位宽4bit
    
    4、USART1配置：
            TX  -->  PA9            50MHz，AF_PP
            波特率：115200
            数据位：8bit
            停止位：1bit
            无校验

    5、测试步骤与现象
        a，测试前请先安装好TF卡
        b，编译下载代码复位运行
        c，从串口看打印信息，验证结果

4、注意事项
    1、在初始化TF卡时，总线位宽必须为1bit，之后根据卡的类型可以用1/4/6bit位宽
    2、读写操作之后必须调用SD_WaitWriteOperation()/SD_WaitReadOperation()/SD_GetStatus()
    以确保操作完成。

1. Function description
	This routine shows the data read and write function between SDIO host interface and TF card.

	First, initialize the SDIO host, and then initialize the TF card.
    
	Write data to TF card, use DMA2_CH4 to transfer Buffer_Block_Tx data to SDIO host, SDIO host will write data to TF card,After DMA completes the write operation, wait for the completion of SDIO host and TF card. If the correct return state is obtained, the write is complete.
    
	Read data from TF card, SDIO host reads data from TF card to data register, and stores data received by SDIO host in Buffer_Block_Rx with DMA2_CH4. Wait for DMA execution to complete, wait for SDIO read operation to complete.
    
	Compare the data consistency between Buffer_Block_Tx and Buffer_Block_Rx, and output the result to the 	serial port

2. Use environment
	Software Development environment:
		IDE tool: KEIL MDK-ARM 5.26.2.0
    
	Hardware environment:
		Development board N32G457-EVB V1.1

3. Instructions for use   
	1. SystemClock: 144MHz
	2. DMA channel: DMA2_CH4
	3. SDIO Configuration:
		D0   -->   PC8          50MHz，AF_PP
		D1   -->   PC9          50MHz，AF_PP
		D2   -->   PC10         50MHz，AF_PP
		D3   -->   PC11         50MHz，AF_PP
		CLK   -->  PC12         50MHz，AF_PP
		CMD   -->  PD2          50MHz，AF_PP
		Frequency division coefficient: 178（SDIOCLK = HCLK, SDIO_CK = HCLK/(2 + divider coefficient)）
		Rising edge effectiveness
		Disable the bypass
		Disable clock hold
		The bus bit width is 4 bits
    
	4. USART1 configuration:
		TX  -->  PA9            50MHz，AF_PP
		Baud rate: 115200
		Data bit: 8 bits
		Stop bit: 1bit
		No check

	5. Test steps and phenomena
		a. Please install the TF card before the test
		b. Compile download code reset run
		c. View the printed information from the serial port and verify the result

4. Precautions
	1. During the initialization of TF card, the bus bit width must be 1bit, and then 1/4/6bit width can be used according to the type of card.
	2. SD_WaitWriteOperation()/SD_WaitReadOperation()/SD_GetStatus() must be called after the read/write operation to ensure that the operation is complete.
1、功能说明

    此例程提供了一种DMA用法，用于在外设到RAM之间传输数据。
             
    初始化CLOCK，GPIO，PERIPH，之后使能SPI的DMA功能，再开启DMA
    
    首先DMA1_CH5将数据从Slave_Tx_Buffer传输到SPI2设备的TX数据寄存器，数据流从SPI2 TX发送
    到SPI1 RX，DMA1_CH4将数据从SPI1的RX寄存器传输到master_rx_buffer。
    
    同时DMA1_CH3将数据从Master_Tx_Buffer传输到SPI1设备的TX数据寄存器，数据流从SPI1 TX发送
    到SPI2 RX，DMA1_CH2将数据从SPI2的RX寄存器传输到slave_rx_buffer。

    等待DMA传输完成，
    比较Slave_Rx_Buffer和Master_Tx_Buffer中的数据一致性，输出对比结果到USART2(TX:PB4)
    比较Master_Rx_Buffer和Slave_Tx_Buffer中的数据一致性，输出对比结果到USART2(TX:PB4)
    

2、使用环境

    软件开发环境：
        IDE工具：KEIL MDK-ARM 5.26.2.0
    
    硬件环境：
        开发板 N32G45XRL-STB V1.1


3、使用说明
    
    1、时钟源：HSE+PLL
    2、主时钟：144MHz
    3、DMA通道：DMA1_CH2，DMA1_CH3，DMA1_CH4，DMA1_CH5
    4、SPI1 配置：
            SCK   -->  PA5          50MHz，AF_PP
            MISO  -->  PA6          50MHz，IN_FLOATING
            MOSI  -->  PA7          50MHz，AF_PP
            全双工
            主模式
            8bit传输
            极性：起始为低/第二个边沿
            软件片选
            大端在前MSB
    
    5、SPI2 配置：
            SCK   -->  PB13         50MHz，IN_FLOATING
            MISO  -->  PB14         50MHz，AF_PP
            MOSI  -->  PB15         50MHz，IN_FLOATING
            全双工
            从模式
            8bit传输
            极性：起始为低/第二个边沿
            软件片选
            大端在前MSB
    
    6、USART1配置：
            TX  -->  PA9            50MHz，AF_PP
            波特率：115200
            数据位：8bit
            停止位：1bit
            无校验

    7、测试步骤与现象
        a，编译下载代码复位运行
        b，从串口看打印信息，验证结果
        
4、注意事项
    无
    
    
1. Function description
    This routine provides a DMA usage for transferring data between peripherals and RAM.
             
    Initialize CLOCK, GPIO, PERIPH, then enable DMA function of SPI, and then DMA
    
    First DMA1_CH5 transfers data from Slave_Tx_Buffer to the TX data register of SPI2 device, and the data stream is sent from SPI2 TX
    To SPI1 RX, DMA1_CH4 transfers data from the RX register of SPI1 to master_rx_buffer.
    
    At the same time, DMA1_CH3 transfers data from Master_Tx_Buffer to the TX data register of SPI1 device, and the data stream is sent from SPI1 TX
    To SPI2 RX, DMA1_CH2 transfers data from SPI2's RX register to Slave_rx_Buffer.
    Wait for the DMA transfer to complete,
    Compare data consistency between Slave_Rx_Buffer and Master_Tx_Buffer, output the comparison result to USART2(TX:PB4)
    Compare Master_Rx_Buffer and Slave_Tx_Buffer to USART2(TX:PB4)
    
2. Use environment
    Software Development environment:
        IDE tool: KEIL MDK-ARM 5.26.2.0
    
    Hardware environment:
        Development board N32G45XRL-STB V1.1
3. Instructions for use    
    1. Clock source: HSE+PLL
    2. Master clock: 144MHz
    3. DMA channels: DMA1_CH2, DMA1_CH3, DMA1_CH4, DMA1_CH5
    4. SPI1 configuration:
	SCK   -->  PA5          50MHz，AF_PP
	MISO  -->  PA6          50MHz，IN_FLOATING
	MOSI  -->  PA7          50MHz，AF_PP
	Full duplex
	Main mode
	8 bit transmission
	Polarity: start at low/second edge
	Piece of software to choose
	Big end in front MSB
    
    5. SPI2 Configuration:
	SCK   -->  PB13         50MHz，IN_FLOATING
	MISO  -->  PB14         50MHz，AF_PP
	MOSI  -->  PB15         50MHz，IN_FLOATING
	Full duplex
	From the pattern
	8 bit transmission
	Polarity: start at low/second edge
	Piece of software to choose
	Big end in front MSB
    
    6. USART1 configuration:
	TX  -->  PA9            50MHz，AF_PP
	Baud rate: 115200
	Data bit: 8 bits
	Stop bit: 1bit
	No check
    7. Test steps and phenomena
	a. Compile download code reset run
	b. View the printed information from the serial port and verify the result
        
4. Matters needing attention
    None

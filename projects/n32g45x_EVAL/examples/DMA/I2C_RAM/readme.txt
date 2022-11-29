1、功能说明

    此例程提供了一种DMA用法，用于在外设与RAM之间传输数据。
             
    初始化CLOCK，GPIO，PERIPH，之后使能I2C的DMA功能，再开启DMA
    
    首先DMA1_CH6将I2C1_Tx_Buffer数据传输到I2C1的数据寄存器，
    然后I2C1发送数据到I2C2
    最后DMA1_CH5将数据从I2C2的数据寄存器传输到I2C2_Rx_Buffer中。
    
    等待DMA传输完成，
    比较I2C1_Tx_Buffer和I2C2_Rx_Buffer中的数据一致性，输出对比结果到串口

2、使用环境

    软件开发环境：
        IDE工具：KEIL MDK-ARM 5.26.2.0
    
    硬件环境：
        开发板 N32G457-EVB


3、使用说明
    
    1、时钟源：HSE+PLL
    2、主时钟：144MHz
    3、DMA通道：DMA1_CH5，DMA1_CH6
    4、I2C1 配置：
            SCL   -->  PB8          50MHz，AF_OD
            SDA   -->  PB9          50MHz，AF_OD
            ADDR：0x30(7bit)
            CLOCK：400K
    
    5、I2C2 配置：
            SCL   -->  PB10          50MHz，AF_OD
            SDA   -->  PB11          50MHz，AF_OD
            ADDR：0x30(7bit)
            CLOCK：400K
    
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
    I2C总线必须上有外接上拉电阻，阻值推荐2.2K~4.7K
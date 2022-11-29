1、功能说明

    此例程提供了一种DMA MemtoMem模式用法，用于在FLASH与RAM之间传输数据。
    
    首先定义了一个静态数组SRC_Const_Buffer，数据保存在FLASH的只读存储区，
    再配置DMA1_CH1通道，用于将数据传输到RAM区DST_Buffer中，
    开启DMA的传输完成中断，用于指示传输已完成，
    
    等待数据传输完成，并比较DST_Buffer的数据与SRC_Const_Buffer中数据是否一致，
    输出对比结果到串口。
    

2、使用环境

    软件开发环境：
        IDE工具：KEIL MDK-ARM 5.26.2.0
    
    硬件环境：
        开发板 N32G457-EVB


3、使用说明
    
    1、时钟源：HSE+PLL
    2、主时钟：144MHz
    3、DMA通道：DMA1_CH1
    
    4、USART1配置：
            TX  -->  PA9            50MHz，AF_PP
            波特率：115200
            数据位：8bit
            停止位：1bit
            无校验

    5、测试步骤与现象
        a，编译下载代码复位运行
        b，从串口看打印信息，验证结果

4、注意事项
    无
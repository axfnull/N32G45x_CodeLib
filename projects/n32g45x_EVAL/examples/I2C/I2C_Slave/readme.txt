1、功能说明

    此例程展示了I2C模块作主设备的读写操作。   

2、使用环境

    软件开发环境：
        IDE工具：KEIL MDK-ARM 5.26.2.0
    
    硬件环境：
        开发板 N32G457-EVB


3、使用说明
    
    1、时钟源：HSE+PLL
    2、主时钟：144MHz
    3、I2C1 配置：
            SCL   -->  PB8          50MHz，AF_OP
            SDA   -->  PB9          50MHz，AF_OP
            ADDR：0x10(7bit)
            CLOCK:100KHz
            
    4、USART1配置：
            TX  -->  PA9            50MHz，AF_PP
            波特率：115200
            数据位：8bit
            停止位：1bit
            无校验

    5、测试步骤与现象
        a，连接I2C主设备
        b，编译下载代码复位运行
        c，从串口看打印信息，验证结果

4、注意事项
    无
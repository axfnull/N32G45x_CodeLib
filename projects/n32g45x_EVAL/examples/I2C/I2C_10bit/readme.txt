1、功能说明

    此例程展示了I2C模块10bit地址模式下的读写操作。

2、使用环境

    软件开发环境：
        IDE工具：KEIL MDK-ARM 5.26.2.0
    
    硬件环境：
        开发板 N32G457QE-EVB


3、使用说明
    
    1、时钟源：HSE+PLL
    2、主时钟：144MHz
    3、I2C1 配置：
            SCL   -->  PB8          50MHz，AF_OP
            SDA   -->  PB9          50MHz，AF_OP
            ADDR：0x230(10bit)
            CLOCK:100KHz

    4、I2C2 配置：
            SCL   -->  PB10          50MHz，AF_OP
            SDA   -->  PB11          50MHz，AF_OP
            ADDR：0x2A0(10bit)
            CLOCK:100KHz           

    5、USART1配置：
            TX  -->  PA9            50MHz，AF_PP
            波特率：115200
            数据位：8bit
            停止位：1bit
            无校验

    6、测试步骤与现象
        a，用杜邦线将I2C1与I2C2连接
        b，编译下载代码复位运行
        c，从串口看打印信息，验证结果

4、注意事项
    无

1. Function description

    This example shows the read and write operations in the 10-bit address mode of the I2C module.

2. Use environment

    Software development environment:
        IDE tool: KEIL MDK-ARM 5.26.2.0
    
    Hardware environment:
        Development board N32G457QE-EVB


3. Instructions for use
    
    1. Clock source: HSE+PLL
    2. Main clock: 144MHz
    3. I2C1 configuration:
            SCL --> PB8 50MHz, AF_OP
            SDA --> PB9 50MHz, AF_OP
            ADDR: 0x230(10bit)
            CLOCK: 100KHz

    4. I2C2 configuration:
            SCL --> PB10 50MHz, AF_OP
            SDA --> PB11 50MHz, AF_OP
            ADDR: 0x2A0(10bit)
            CLOCK: 100KHz

    5. USART1 configuration:
            TX --> PA9 50MHz, AF_PP
            Baud rate: 115200
            Data bit: 8bit
            Stop bit: 1bit
            No verification

    6. Test steps and phenomena
        a. Connect I2C1 and I2C2 with DuPont cable
        b, compile and download the code, reset and run
        c, view the print information from the serial port and verify the result

4. Matters needing attention
    without
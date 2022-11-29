1、功能说明

    此例程展示了通过I2C模块与外部EEPRON的通信。   

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

            ADDR：0xA0(7bit)
            CLOCK:400KHz
            
    4、USART1配置：
            TX  -->  PA9            50MHz，AF_PP
            波特率：115200
            数据位：8bit
            停止位：1bit
            无校验

    5、测试步骤与现象
        a，检查EEPROM连接
        b，编译下载代码复位运行
        c，从串口看打印信息，验证结果

4、注意事项
    1，此处使用的EEPROM是AT24C02，32个page，每个page 8byte
    2，读写数据时若长度大于一个page，则器件地址会自动回卷

1. Function description

    This example demonstrates the communication with the external EEPROM through the I2C module.

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

            ADDR: 0xA0(7bit)
            CLOCK: 400KHz
            
    4. USART1 configuration:
            TX --> PA9 50MHz, AF_PP
            Baud rate: 115200
            Data bit: 8bit
            Stop bit: 1bit
            No verification

    5. Test steps and phenomena
        a, check the EEPROM connection
        b, compile and download the code, reset and run
        c, view the print information from the serial port and verify the result

4. Matters needing attention
    1. The EEPROM used here is AT24C02, 32 pages, 8 bytes per page
    2. When reading and writing data, if the length is greater than one page, the device address will be automatically rolled back
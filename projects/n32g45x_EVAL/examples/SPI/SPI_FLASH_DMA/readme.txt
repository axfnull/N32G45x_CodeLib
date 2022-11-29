1、功能说明

    1、SPI DMA 读、写 W25Q128 数据

2、使用环境

    /* 硬件环境：工程对应的开发硬件平台 */
    开发板：N32G457QE_EVB V1.1

3、使用说明
    
    /* 描述相关模块配置方法；例如:时钟，I/O等 */
    1、SystemClock：144MHz
    2、GPIO：SPI1: NSS--PA4、SCK--PA5、MISO--PA6、MOSI--PA7
    3、USART: TX--PA9,115200,8bit data,1bit stop
    
    /* 描述Demo的测试步骤和现象 */
    1、编译后下载程序复位运行；
    2、通过 SPI1 读取 W25Q128 的 ID，然后通过 DMA 写数据，再通过 DMA 读出来，比较读写数据，
         查看 TransferStatus1 状态为 PASSED，然后擦除块，检查擦除块正常。
    3、通过串口工具查看结果

4、注意事项
    只在大批量读写数据时使用 DMA，刚开始因为涉及到一些写命令，所以配置成全双工，在全双工模式下，读取 Flash 需要一直发送 0xFF，这样会限制 DMA 性能，
    所以在用 DMA 读取数据时，设置 SPI 为只读模式，这样在读取数据时不需要一直发送 0xFF，以此来提升性能。




1. Function description

    1. SPI DMA read and write W25Q128 data

2. Use environment

    /* Hardware environment: the development hardware platform corresponding to the project */
    Development board: N32G457QE_EVB V1.1

3. Instructions for use
    
    /* Describe related module configuration methods; for example: clock, I/O, etc. */
    1. SystemClock: 144MHz
    2. GPIO: SPI1: NSS--PA4, SCK--PA5, MISO--PA6, MOSI--PA7
    3. USART: TX--PA9, 115200, 8bit data, 1bit stop
    
    /* Describe the test steps and phenomena of the Demo */
    1. After compiling, download the program to reset and run;
    2. Read the ID of W25Q128 through SPI1, then write the data through DMA, and then read it out through DMA,
       compare the read and write data, check that the status of TransferStatus1 is PASSED, then erase the block,
       and check that the erase block is normal.
    3. View the results through the serial port tool

4. Matters needing attention
    Only use DMA when reading and writing data in large batches. At the beginning, because some write commands are involved,
    it is configured as full-duplex. In full-duplex mode, 0xFF needs to be sent all the time to read Flash, 
    which will limit the DMA performance. When DMA reads data, set SPI to read-only mode,
    so that 0xFF does not need to be sent all the time when reading data, so as to improve performance.
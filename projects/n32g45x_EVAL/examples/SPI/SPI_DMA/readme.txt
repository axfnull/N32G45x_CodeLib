1、功能说明

    1、SPI DMA 单线接收数据

2、使用环境

    /* 硬件环境：工程对应的开发硬件平台 */
    开发板：N32G45XV-STB V1.1

3、使用说明
    
    /* 描述相关模块配置方法；例如:时钟，I/O等 */
    1、SystemClock：144MHz
    2、GPIO：SPI1: NSS--PA4、SCK--PA5、 MOSI--PA7,
                    SPI2: NSS--PB12、SCK--PB13、MISO--PB14
    3、中断：SPI1 中断入口函数 SPI1_IRQHandler，SPI2 中断入口函数 SPI2_IRQHandler 
    4、DMA：DMA1_CH4 通道接收数据

    /* 描述Demo的测试步骤和现象 */
    1、编译后下载程序复位运行；
    2、SPI1 发送数据，SPI2 通过 DMA 接收数据，数据传输完成后，查看 TransferStatus 状态为 PASSED；

4、注意事项
    “单线”数据线在主设备端为MOSI引脚，在从设备端为MISO引脚


1. Function description

     1. SPI DMA single-wire receive data

2. Use environment

     /* Hardware environment: the development hardware platform corresponding to the project */
     Development board: N32G45XV-STB V1.1

3. Instructions for use
    
     /* Describe related module configuration methods; for example: clock, I/O, etc. */
     1. SystemClock: 144MHz
     2. GPIO: SPI1: NSS--PA4, SCK--PA5, MOSI--PA7,
                     SPI2: NSS--PB12, SCK--PB13, MISO--PB14
     3. Interrupt: SPI1 interrupt entry function SPI1_IRQHandler, SPI2 interrupt entry function SPI2_IRQHandler
     4. DMA: DMA1_CH4 channel receives data

     /* Describe the test steps and phenomena of the Demo */
     1. After compiling, download the program to reset and run;
     2. SPI1 sends data, and SPI2 receives data through DMA. After the data transfer is completed, check that the TransferStatus status is PASSED;

4. Matters needing attention
     The "single wire" data lines are MOSI pins on the master side and MISO pins on the slave side
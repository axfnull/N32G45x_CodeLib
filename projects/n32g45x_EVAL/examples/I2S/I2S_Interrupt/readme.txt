1、功能说明

    1、I2S 通过中断收发数据

2、使用环境

    /* 硬件环境：工程对应的开发硬件平台 */
    开发板：N32G4XM-STB V1.1

3、使用说明
    
    /* 描述相关模块配置方法；例如:时钟，I/O等 */
    1、SystemClock：144MHz
    2、GPIO：I2S2（SPI2 WS--PB12、SCK--PB13、SD--PB15），
                    I2S3（SPI3 WS--PA15、SCK--PB3、 SD--PB5）,
    3、中断：I2S2 中断入口函数 SPI2_IRQHandler，I2S3 中断入口函数SPI3_IRQHandler，

    /* 描述Demo的测试步骤和现象 */
    1、编译后下载程序复位运行；
    2、I2S 初始化成 16 位数据格式，48K 采样频率，有数据接收时进入 SPI3_IRQHandler 函数接收数据，I2S2 有数据发送时进入 SPI2_IRQHandler 函数发送数据 ，运行完成后，查看 TransferStatus1 状态为 PASSED；
         I23 初始化成 24 位数据格式，16K 采样频率，有数据接收时进入 SPI3_IRQHandler 函数接收数据，I2S2 有数据发送时进入 SPI2_IRQHandler 函数发送数据 ，运行完成后，查看 TransferStatus2 状态为 PASSED；

4、注意事项
    无



1. Function description

    1. I2S sends and receives data through interrupts

2. Use environment

    /* Hardware environment: the development hardware platform corresponding to the project */
    Development board: N32G4XM-STB V1.1

3. Instructions for use
    
    /* Describe related module configuration methods; for example: clock, I/O, etc. */
    1. SystemClock: 144MHz
    2. GPIO: I2S2 (SPI2 WS--PB12, SCK--PB13, SD--PB15),
                 I2S3 (SPI3 WS--PA15, SCK--PB3, SD--PB5),
    3. Interrupt: I2S2 interrupt entry function SPI2_IRQHandler, I2S3 interrupt entry function SPI3_IRQHandler,

    /* Describe the test steps and phenomena of the Demo */
    1. After compiling, download the program to reset and run;
    2. I2S is initialized to 16-bit data format, 48K sampling frequency. When data is received, it enters the SPI3_IRQHandler function to receive data. When I2S2 has data to send, it enters the SPI2_IRQHandler function to send data. After the operation is completed, check that the status of TransferStatus1 is PASSED;
        I23 is initialized to 24-bit data format, 16K sampling frequency. When data is received, it enters the SPI3_IRQHandler function to receive data. When I2S2 has data to send, it enters the SPI2_IRQHandler function to send data. After the operation is completed, check that the status of TransferStatus2 is PASSED;

4. Matters needing attention
    none
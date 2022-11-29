1、功能说明

    1、SPI 重映射后发送接收数据进行 CRC 校验

2、使用环境

    /* 硬件环境：工程对应的开发硬件平台 */
    开发板：N32G45XV-STB V1.1

3、使用说明
    
    /* 描述相关模块配置方法；例如:时钟，I/O等 */
    1、SystemClock：144MHz
    2、GPIO：SPI1: SCK--PB3、MISO--PB4、MOSI--PB5,
                    SPI2: SCK--PC7、MISO--PC8、MOSI--PC9，

    /* 描述Demo的测试步骤和现象 */
    1.编译后下载程序复位运行；
    2.SPI1 和 SPI2 重映射初始化完成以后，SPI1 发送数据，SPI2 接收数据 ，传输完成后，发送 CRC 数据，检查数据和 CRC 值，查看 TransferStatus1 和 TransferStatus2 状态为 PASSED，
      再查看 CRC 值；

4、注意事项
    无



1. Function description

     1. After SPI remapping, send and receive data for CRC check

2. Use environment

     /* Hardware environment: the development hardware platform corresponding to the project */
     Development board: N32G45XV-STB V1.1

3. Instructions for use
    
     /* Describe related module configuration methods; for example: clock, I/O, etc. */
     1. SystemClock: 144MHz
     2. GPIO: SPI1: SCK--PB3, MISO--PB4, MOSI--PB5,
                  SPI2: SCK--PC7, MISO--PC8, MOSI--PC9,

     /* Describe the test steps and phenomena of the Demo */
     1. After compiling, download the program to reset and run;
     2. After the remapping initialization of SPI1 and SPI2 is completed, SPI1 sends data, SPI2 receives data, after the transmission is completed, send CRC data, check the data and CRC value, check that the status of TransferStatus1 and TransferStatus2 is PASSED,
        Check the CRC value again;

4. Matters needing attention
     none
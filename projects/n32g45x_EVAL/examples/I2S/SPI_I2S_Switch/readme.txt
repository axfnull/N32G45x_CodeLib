1、功能说明

    1、I2S 收发数据完成以后切换成 SPI 收发数据，再切换成 I2S 收发数据


2、使用环境

    /* 硬件环境：工程对应的开发硬件平台 */
    开发板：N32G4XM-STB V1.1

3、使用说明
    
    /* 描述相关模块配置方法；例如:时钟，I/O等 */
    1、SystemClock：144MHz
    2、GPIO：I2S2（SPI2 WS--PB12、SCK--PB13、MOSI--PB15），
             I2S3（SPI3 WS--PA15、SCK--PB3、 MOSI--PB5）


    /* 描述Demo的测试步骤和现象 */
    1.编译后下载程序复位运行；
    2.I2S2 和 I2S3 初始化，I2S3 发送数据，I2S2 接收数据 ，运行完成后，查看 TransferStatus1 状态为 PASSED；
      SPI2 和 SPI3 初始化，SPI3 发送数据，SPI2 接收数据 ，运行完成后，查看 TransferStatus2 状态为 PASSED；
      I2S2 和 I2S3 初始化，I2S3 发送数据，I2S2 接收数据 ，运行完成后，查看 TransferStatus3 状态为 PASSED；

4、注意事项
    无
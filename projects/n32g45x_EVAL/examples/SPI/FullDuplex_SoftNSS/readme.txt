1、功能说明

    1、SPI 全双工软件 NSS 模式发送接收数据


2、使用环境

    /* 硬件环境：工程对应的开发硬件平台 */
    开发板：N32G45XV-STB V1.1

3、使用说明
    
    /* 描述相关模块配置方法；例如:时钟，I/O等 */
    1、SystemClock：144MHz
    2、GPIO：SPI1: SCK--PA5、 MISO--PA6、MOSI--PA7,
             SPI2: SCK--PB13、MISO--PB14、MOSI--PB15，

    /* 描述Demo的测试步骤和现象 */
    1.编译后下载程序复位运行；
    2.SPI1 初始化位主机，发送数据，SPI2 初始化位丛机，接收数据 ，传输完成后，检查数据，查看 TransferStatus1 和 TransferStatus2 状态为 PASSED，
      SPI2 初始化位主机，发送数据，SPI1 初始化位丛机，接收数据 ，传输完成后，检查数据，查看 TransferStatus3 和 TransferStatus4 状态为 PASSED；

4、注意事项
    无
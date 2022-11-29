1、功能说明

    1、SPI DMA 单线发送和单线接收数据


2、使用环境

    /* 软件开发环境：当前工程使用的软件工具名称及版本号 */
    IDE工具：KEIL MDK-ARM 5.26.2.0

    /* 硬件环境：工程对应的开发硬件平台 */
    开发板：N32G45XV-STB V1.1

3、使用说明
    
    /* 描述相关模块配置方法；例如:时钟，I/O等 */
    1、SystemClock：144MHz
    2、GPIO：SPI1: NSS--PA4、SCK--PA5、 MOSI--PA7,
             SPI2: NSS--PB12、SCK--PB13、MISO--PB14
    3、DMA：DMA1_CH3 通道发送数据，DMA1_CH4 通道接收数据

    /* 描述Demo的测试步骤和现象 */
    1.编译后下载程序复位运行；
    2.SPI1 通过 DMA 发送数据，SPI2 通过 DMA 接收数据，数据传输完成后，查看 TransferStatus 状态为 PASSED；

4、注意事项
    “单线”数据线在主设备端为MOSI引脚，在从设备端为MISO引脚。
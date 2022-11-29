1、功能说明

    1、SPI 读、写、擦除 W25Q128


2、使用环境

    /* 硬件环境：工程对应的开发硬件平台 */
    开发板：N32G457QE_EVB V1.1

3、使用说明
    
    /* 描述相关模块配置方法；例如:时钟，I/O等 */
    1、SystemClock：144MHz
    2、GPIO：SPI1: NSS--PA4、SCK--PA5、MISO--PA6、MOSI--PA7
    3、USART: TX--PA9,115200,8bit data,1bit stop

    /* 描述Demo的测试步骤和现象 */
    1.编译后下载程序复位运行；
    2.通过串口工具查看结果。

4、注意事项
    无
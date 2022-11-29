1、功能说明

    1、QSPI XIP 内存映射模式 擦除、读、写 P25Q40HA

2、使用环境

        /* 硬件环境：工程对应的开发硬件平台 */
        开发板：N32G457QE_EVB V1.0

3、使用说明
    
    /* 描述相关模块配置方法；例如:时钟，I/O等 */
        1、SystemClock：144MHz
        2、GPIO：QSPI（NSS--PA4、SCK--PA5、IO0--PA6、IO1--PA7、IO2--PC4、IO3--PC5）   
        3、DMA：QSPI 接收采用 DMA2_CH6 通道，QSPI 发送采用 DMA2_CH7 通道

    /* 描述Demo的测试步骤和现象 */
        1、编译后下载程序复位运行；
        2、QSPI 初始化完成后，先擦除 P25Q40HA 一个扇区，通过日志打印检查擦除正常，再往 P25Q40HA 写数据，
              写完后再读出来，比较读写数据，日志打印读写正常，再用日志打印读出来的数据；

4、注意事项
    N32G457QE_EVB V1.0 开发板使用 QSPI 需要将跳线 J1、J2、J8、J12、J15、J20 连接起来，并断开 J29、J30、J31、J32



1. Function description

    1. QSPI XIP memory mapping mode erase, read, write P25Q40HA

2. Use environment

        /* Hardware environment: the development hardware platform corresponding to the project */
        Development board: N32G457QE_EVB V1.0

3. Instructions for use
    
    /* Describe related module configuration methods; for example: clock, I/O, etc. */
        1. SystemClock: 144MHz
        2. GPIO: QSPI (NSS--PA4, SCK--PA5, IO0--PA6, IO1--PA7, IO2--PC4, IO3--PC5)
        3. DMA: QSPI reception adopts DMA2_CH6 channel, QSPI transmission adopts DMA2_CH7 channel

    /* Describe the test steps and phenomena of the Demo */
        1. After compiling, download the program to reset and run;
        2. After the QSPI initialization is completed, first erase a sector of P25Q40HA, check the erasure is normal through log printing, and then write data to P25Q40HA,
              After writing, read it out, compare the read and write data, the log print and read and write are normal, and then use the log to print the read data;

4. Matters needing attention
    For N32G457QE_EVB V1.0 development board to use QSPI, you need to connect jumpers J1, J2, J8, J12, J15, J20, and disconnect J29, J30, J31, J32

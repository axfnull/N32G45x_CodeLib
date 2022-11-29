1、功能说明
    1、通过DVP接口从摄像头采集数据，并在TFT_LCD上实时显示。
    2、支持单色和彩色模式，通过DVP_demo.h中的宏 DVP_IMAGE_FORMAT 来选择
            DVP_IMAGE_FORMAT = DVP_IMAGE_FORMAT_GRAY:  单色
            DVP_IMAGE_FORMAT = DVP_IMAGE_FORMAT_RGB565：彩色

2、使用环境
    软件开发环境：KEIL MDK-ARM V5.26
    
    硬件环境：
        1、基于全功能板N32G457QEL-EVB开发
        2、摄像头：GC0308（640*480）
        3、TFT_LCD（320*240 驱动：ILI9341）

3、使用说明
    系统配置；
        1、时钟源：HSE+PLL
        2、系统时钟：144MHz
        3、DMA：DMA2通道8
        4、MCO：PA8
        5、DVP端口：PE2:HSYNC   PE3:VSYNC   PE4:PCLK
                    PE5:D0      PE6:D1      PC0:D2      PB2:D3
                    PF12:D4     PF13:D5     PF14:D6     PF15:D7
        6、摄像头控制端口：PA8:MCLK  PA15:RESET  PB7:PWDN  PB8:SCL  PB9:SDA
        7、ILI9341端口:
                CS:PB7      DC:PG0      WR:PD5      RD:PD4      
                D0:PD14     D1:PD15     D2:PD0      D3:PD1      
                D4:PE7      D5:PE8      D6:PE9      D7:PE10     
                D8:PE11     D9:PE12     D10:PE13    D11:PE14    
                D12:PE15    D13:PD8     D14:PD9     D15:PD10
        8、LCD背光控制端口：PD6

    使用方法：
        在KEIL下编译后烧录到全功能板，通电，可在LCD上实时看到摄像头捕获的图像。
        切换单色和彩色模式需要修改宏DVP_IMAGE_FORMAT后重新编译烧录。


4、注意事项
    无
1、  功能说明

    /* 简单描述工程功能 */
    1、USB  Joystick Mouse 设备

2、  使用环境

    /* 软件开发环境：当前工程使用的软件工具名称及版本号 */
    IDE工具：KEIL MDK-ARM 5.26.2.0

    /* 硬件环境：工程对应的开发硬件平台 */
    开发板：N32G4XM_STB V1.1

3、  使用说明

    /* 描述相关模块配置方法；例如:时钟，I/O等 */
    1、SystemClock：144MHz
    2、USBClock：48MHz
    3、KEY：右KEY1（PA4）、左KEY2（PA5）、上KEY3（PA6）、下KEY4（PA7）

    /* 描述Demo的测试步骤和现象 */
    1.编译后下载程序复位运行；
    2.通过 USB 线连接 J3 USB 口，按下 KEY1、KEY2、KEY3、KEY4 鼠标会上下左右移动；

4、注意事项
    无
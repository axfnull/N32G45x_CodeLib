1. 功能说明
    USB HID 键盘设备双向传输

2. 使用环境
    硬件环境：工程对应的开发硬件平台 
    开发板：   N32G4XV-STB V1.1

3. 使用说明
    描述相关模块配置方法；例如:时钟，I/O等 
         1. SystemClock：144MHz
         2. USBClock: 48MHz
         3. GPIO：
             KEY1(PA4)、KEY2(PA5)、KEY3(PA6) 键盘输入 A、B、C；
             PA8控制 LED1(D1)，PB5控制 LED2(D3)；
             LED1(D1)显示 NUMLOCK 灯，LED2(D3)显示 CAPSLOCK 灯。

    描述Demo的测试步骤和现象 
         1. 编译后下载程序复位运行；
         2. 通过 USB 线连接 J3 USB 口，电脑识别出键盘设备，按下 KEY1、KEY2、KEY3 分别输入 A、B、C，PC 接入其他键盘，按下 CAPSLOCK 和 NUMLOCK，D1、D3 分别对应亮灭。

4. 注意事项
    无

1. Function description
    Two-way transmission of USB HID keyboard device

2. Use environment
    Hardware environment: development hardware platform corresponding to the project 
    Development board:      N32G4XV-STB V1.1

3. Instructions for use
    Describe the configuration method of related modules; for example: clock, I/O, etc. 
        1. SystemClock: 144MHz
        2. USBClock: 48MHz
        3. GPIO: 
            KEY1 (PA4), KEY2 (PA5), KEY3 (PA6) keyboard input A, B, C;
            PA8 controls LED1 (D1), PB5 controls LED2 (D3);
            LED1 (D1) displays NUMLOCK light, LED2 (D3) displays CAPSLOCK light.
                  
    Describe the test steps and phenomena of Demo 
        1. After compiling, download the program to reset and run;
        2. Connect the J3 USB port through the USB cable, the computer recognizes the keyboard device, press KEY1, KEY2, and KEY3 to input A, B, C respectively, 
            and the PC connects to other keyboards, press CAPSLOCK and NUMLOCK, D1 and D3 are respectively on and off.
 
4. Matters needing attention
    None.
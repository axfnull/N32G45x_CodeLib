1. 功能说明
    USB  模拟串口回环功能

2. 使用环境
    硬件环境：工程对应的开发硬件平台 
    开发板：   N32G45XV-STB V1.1

3. 使用说明
    描述相关模块配置方法；例如:时钟，I/O等 
         1. SystemClock：144MHz
         2. USBClock: 48MHz

    描述Demo的测试步骤和现象 
         1. 编译后下载程序复位运行；
         2. 通过 USB 线连接 J3 USB 口，USB 挂载完成后，在电脑设备管理中可以看到新增的串口，打开 USB 虚拟串口，USB 虚拟串口发送数据后会立马会接收到刚发送的数据

4. 注意事项
    无

1. Function description
    USB virtual serial port loopback function

2. Use environment
    Hardware environment: development hardware platform corresponding to the project 
    Development board:      N32G45XV-STB V1.1

3. Instructions for use
    Describe the configuration method of related modules; for example: clock, I/O, etc. 
        1. SystemClock: 144MHz
        2. USBClock: 48MHz
                  
    Describe the test steps and phenomena of Demo 
        1. After compiling, download the program to reset and run;
        2. Connect the J3 USB port via a USB cable. After the USB is mounted, you can see the newly added serial port in the computer device management. 
           Open the USB virtual serial port. After the USB virtual serial port sends data, it will immediately receive the data just sent.
 
4. Matters needing attention
    None.
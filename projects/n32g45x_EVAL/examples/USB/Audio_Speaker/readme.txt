1. 功能说明
    USB Audio Speaker 声卡

2. 使用环境
    硬件环境：工程对应的开发硬件平台 
    开发板：   N32G457QE_EVB V1.0

3. 使用说明
    描述相关模块配置方法；例如:时钟，I/O等 
         1. SystemClock：144MHz
         2. USBClock: 48MHz
         3. GPIO: PB1 作为 PWM_DAC 输出，同时全功能板的 J22 用跳线帽连接，用耳机接入 J21

    描述Demo的测试步骤和现象 
         1. 编译后下载程序复位运行；
         2. 通过 USB 线连接 J3 USB 口，电脑识别出声卡设备，电脑播放设备选择刚识别出来的声卡，用 Bus Hound 查看电脑传过去的数据，音频数据是 ISOC 同步传输类型，播放音乐，耳机可以听到电脑播放的音乐。

4. 注意事项
    此例程仅在N32G457QE_EVB V1.0硬件版本上支持

1. Function description
    USB Audio Speaker

2. Use environment
    Hardware environment: development hardware platform corresponding to the project 
    Development board:      N32G457QE_EVB V1.0

3. Instructions for use
    Describe the configuration method of related modules; for example: clock, I/O, etc. 
        1. SystemClock: 144MHz
        2. USBClock: 48MHz
        3. GPIO: PB1 is used as PWM_DAC output, and J22 of the full-function board is connected with a jumper cap and connected to J21 with a headset.
                  
    Describe the test steps and phenomena of Demo 
        1. After compiling, download the program to reset and run;
        2. Connect the J3 USB port through the USB cable, the computer recognizes the sound card device, and the computer playback device selects the sound card that has just been recognized. 
        Use Bus Hound to view the data transmitted by the computer. The audio data is ISOC synchronous transmission type, playing music, headphones can hear the music played by the computer.

4. Matters needing attention
    This routine is only supported on the N32G457QE_EVB V1.0 hardware version.
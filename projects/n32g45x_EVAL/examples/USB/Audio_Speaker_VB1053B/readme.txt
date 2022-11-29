1. 功能说明
    USB Audio Speaker 声卡

2. 使用环境
    硬件环境：工程对应的开发硬件平台 
    开发板：   N32G457QEL_EVB V1.1

3. 使用说明
    描述相关模块配置方法；例如:时钟，I/O等 
         1. SystemClock：144MHz
         2. USBClock: 48MHz
         3. GPIO: VS1053B SPI3：SCK--PC3、MISO--PA0、MOSI--PA1；DREQ--PA4、RST--PB0、XCS--PC2、XDCS--PA3，J46、J47需要用跳线帽连接，耳机接在J21耳机口上。

    描述Demo的测试步骤和现象 
         1. 编译后下载程序复位运行；
         2. 通过 USB 线连接 J3 USB 口，电脑识别出声卡设备，电脑播放设备选择刚识别出来的声卡，USB获取音频数据后发送到VS1053B上，通过VS1053B进行解码。
            用 Bus Hound 查看电脑传过去的数据，音频数据是 ISOC 同步传输类型，播放音乐，耳机可以听到电脑播放的音乐。
 
4. 注意事项
    本例程音频为单声道，只有左耳机可以听到声音。

1. Function description
    USB Audio Speaker

2. Use environment
    Hardware environment: development hardware platform corresponding to the project 
    Development board:      N32G457QEL_EVB V1.1

3. Instructions for use
    Describe the configuration method of related modules; for example: clock, I/O, etc. 
        1. SystemClock: 144MHz
        2. USBClock: 48MHz
        3. GPIO: 
            VS1053B SPI3: SCK--PC3, MISO--PA0, MOSI--PA1; 
            DREQ--PA4, RST--PB0, XCS--PC2, XDCS--PA3, 
            J46 and J47 need to be connected by jumper cap, and the earphone is connected to the earphone port of J21.
                  
    Describe the test steps and phenomena of Demo 
        1. After compiling, download the program to reset and run;
        2. Connect the J3 USB port with the USB cable, the computer recognizes the sound card device, the computer player selects the newly recognized sound card, 
            USB obtains the audio data and sends it to VS1053B, through which VS1053B decodes.
            Use Bus Hound to check the data transmitted by the computer. The audio data is ISOC synchronous transmission type. When playing music, the headset can hear the music played by the computer.
 
4. Matters needing attention
    This example audio is mono, only the left earphone can hear the sound.
1、功能说明

    1、SPI 控制 CODEC 模块进行录音和播放录音

2、使用环境

    /* 硬件环境：工程对应的开发硬件平台 */
    开发板：N32G457QE_EVB V1.1

3、使用说明
    
    /* 描述相关模块配置方法；例如:时钟，I/O等 */
    1、SystemClock：144MHz
    2、GPIO：W25Q128 SPI1: NSS--PD6、SCK--PA5、MISO--PA6、MOSI--PA7
                     VS1053B SPI3：SCK--PC3、MISO--PA0、MOSI--PA1；DREQ--PA4、RST--PB0、XCS--PC2、XDCS--PA3

    3、按键：S4：KEY0--PC6（开始/暂停录音）、S5：KEY1--PC7（保存录音）、S6：KEY2--PB6（播放录音）
    4、日志打印：TX--PA9    波特率：115200
    5、J29、J31、J32、J41、J42、J43、J46、J47需要用跳线帽连接，W25Q128 的 NSS 引脚由于和 VS1053B 的 DREQ 引脚重了，
         所以用 PD3 作为 W25Q128 的 NSS 引脚，需要用跳线连接。

    /* 描述Demo的测试步骤和现象 */
    1、编译后下载程序复位运行；
    2、先依次播放固定频率的测试正弦波，分别为6K、1K，各持续约3s，然后进入录音测试状态
    2、通过串口工具查看日志，按下 S4 按键开始录音，按下 S5 保存录音，按下 S6 播放录音

4、注意事项
    1、录音数据在录音过程中保存到 W25Q128，播放录音时先从 W25Q128 读取录音数据，然后发送到 VS1053B 音频芯片进行解码，用耳机可以听到录音；
    2、录音之前会先删除之前的录音数据（每次录音数据保存的起始地址是相同的）；
    3、正弦测试需要插入耳机才能听到，录音时耳机要远离麦克风，否则可能自激啸叫；

    

1. Function description

    1. SPI controls the CODEC module to record and play the recording

2. Use environment

    /* Hardware environment: the development hardware platform corresponding to the project */
    Development board: N32G457QE_EVB V1.1

3. Instructions for use
    
    /* Describe related module configuration methods; for example: clock, I/O, etc. */
    1. SystemClock: 144MHz
    2. GPIO: W25Q128 SPI1: NSS--PD6, SCK--PA5, MISO--PA6, MOSI--PA7
                     VS1053B SPI3: SCK--PC3, MISO--PA0, MOSI--PA1; DREQ--PA4, RST--PB0, XCS--PC2, XDCS--PA3

    3. Buttons: S4: KEY0--PC6 (start/pause recording), S5: KEY1--PC7 (save recording), S6: KEY2--PB6 (play recording)
    4. Log printing: TX--PA9 Baud rate: 115200
    5. J29, J31, J32, J41, J42, J43, J46, J47 need to be connected with jumper caps. 
       Because the NSS pin of W25Q128 is heavy with the DREQ pin of VS1053B, PD3 is used as the NSS pin of W25Q128. Connect with jumper.

    /* Describe the test steps and phenomena of the Demo */
    1. After compiling, download the program to reset and run;
    2. First play the test sine wave of fixed frequency, 6K and 1K respectively, each lasting about 3s, and then enter the recording test state
    2. View the log through the serial port tool, press the S4 button to start recording, press S5 to save the recording, and press S6 to play the recording

4. Matters needing attention
    1. The recording data is saved to the W25Q128 during the recording process. 
        When playing the recording, the recording data is first read from the W25Q128, 
        and then sent to the VS1053B audio chip for decoding. The recording can be heard with headphones;
    2. Before recording, the previous recording data will be deleted (the starting address of each recording data is the same);
    3. The sine test needs to be plugged in to hear it. The headset should be kept away from the microphone when recording,
        otherwise it may self-excited howling;
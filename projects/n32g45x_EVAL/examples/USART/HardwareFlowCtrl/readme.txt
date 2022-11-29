1、功能说明

    该测例演示了USARTy与USARTz间使用硬件流控制的基础通信。USARTy和
USARTz可以是USART1和USART2。
    首先，USARTy利用CTS发送TxBuffer1数据，USARTz利用RTS接收数据，存至
RxBuffer2；USARTz利用CTS发送TxBuffer2数据，USARTy利用RTS接收数据，存至
RxBuffer1。
    随后，比较接收数据与发送数据，比较结果分别存入TransferStatus1和
TransferStatus2变量。


2、使用环境

        软件开发环境：KEIL MDK-ARM Professional Version 5.26.2.0

        硬件环境：最小系统板N32G45XV-STB_V1.1


3、使用说明
    
    系统时钟配置如下：
    - 时钟源 = HSE + PLL
    - 系统时钟 = 144MHz
    
    USARTy配置如下：
    - 波特率 = 115200 baud
    - 字长 = 8数据位
    - 1停止位
    - 校验控制禁用
    - CTS和RTS硬件流控制使能
    - 接收器和发送器使能   
    
    USARTz配置如下：
    - 波特率 = 115200 baud
    - 字长 = 8数据位
    - 1停止位
    - 校验控制禁用
    - CTS和RTS硬件流控制使能
    - 接收器和发送器使能   
    
    
    USART引脚连接如下：    
    - USART1_Tx.PA9      <------->   USART2_Rx.PA3
    - USART1_Rx.PA10    <------->   USART2_Tx.PA2
    - USART1_CTS.PA11  <------->   USART2_RTS.PA1 
    - USART1_RTS.PA12  <------->   USART2_CTS.PA0    

    
    测试步骤与现象：
    - Demo在KEIL环境下编译后，下载至MCU
    - 复位运行，查看变量TransferStatus1和TransferStatus2，其中，PASSED为测试通过，
      FAILED为测试异常


4、注意事项
    使用PA9，PA10需要把开发板NS-LINK的MCU_TX和MCU_RX跳线帽断开

1. Function description

    This test example demonstrates the basic communication between USARTy and USARTz using hardware flow control. USARTy and
USARTz can be USART1 and USART2.
    First, USARTy uses CTS to send TxBuffer1 data, and USARTz uses RTS to receive data and save it to
RxBuffer2; USARTz uses CTS to send TxBuffer2 data, and USARTy uses RTS to receive data and save it to
RxBuffer1.
    Subsequently, compare the received data with the sent data, and the comparison results are stored in TransferStatus1 and
The TransferStatus2 variable.


2. Use environment

        Software development environment: KEIL MDK-ARM Professional Version 5.26.2.0

        Hardware environment: minimum system board N32G45XV-STB_V1.1


3. Instructions for use
    
    The system clock configuration is as follows:
    -Clock source = HSE + PLL
    -System clock = 144MHz
    
    The USARTy configuration is as follows:
    -Baud rate = 115200 baud
    -Word length = 8 data bits
    -1 stop bit
    -Parity control disabled
    -CTS and RTS hardware flow control enable
    -Receiver and transmitter enable
    
    The USARTz configuration is as follows:
    -Baud rate = 115200 baud
    -Word length = 8 data bits
    -1 stop bit
    -Verification control disabled
    -CTS and RTS hardware flow control enable
    -Receiver and transmitter enable
    
    
    The USART pin connections are as follows:
    -USART1_Tx.PA9 <-------> USART2_Rx.PA3
    -USART1_Rx.PA10 <-------> USART2_Tx.PA2
    -USART1_CTS.PA11 <-------> USART2_RTS.PA1
    -USART1_RTS.PA12 <-------> USART2_CTS.PA0

    
    Test steps and phenomena:
    -After the Demo is compiled in the KEIL environment, download it to the MCU
    -Reset operation, check the variables TransferStatus1 and TransferStatus2, among them, PASSED means the test passed,
      FAILED is test abnormal


4. Matters needing attention
    To use PA9/PA10, disconnect the jumper cap of MCU_TX/MCU_RX on NS-LINK
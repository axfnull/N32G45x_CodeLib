1、功能说明

    该测例演示了USARTy与USARTz间实现串行IrDA低功耗模式红外解码功能的基础通信。
    首先，USARTy发送TxBuffer1数据至USARTz，USARTz通过中断接收数据存至RxBuffer1。
    随后，比较接收数据与发送数据，比较结果存入TransferStatus变量。
    


2、使用环境

    软件开发环境：KEIL MDK-ARM Professional Version 5.26.2.0

        硬件环境：开发板N32G457QEL_EVB_V1.1，IrDA接收器、发送器


3、使用说明
    
    系统时钟配置如下：
    - 时钟源 = HSE + PLL
    - 系统时钟 = 8MHz
    
    USART配置如下：
    - 波特率 = 600 baud
    - 字长 = 8数据位
    - 1停止位
    - 校验控制禁用
    - 硬件流控制禁用（RTS和CTS信号）
    - 接收器和发送器使能
    - IrDA模式使能
    
    USART引脚连接如下：
    - USART1_Tx.PA9    <------->   IrDA Transmitter
    - USART2_Rx.PB5    <------->   IrDA Receiver

    
    - GPIO.PD3       <------->    38kHz carrier

    
    测试步骤与现象：
    -在KEIL环境下编译后，下载至MCU
    -运行查看变量TransferStatus，其中，PASSED为测试通过，FAILED为测试异常


4、注意事项
    使用PA9，PA10需要把开发板NS-LINK的MCU_TX和MCU_RX跳线帽断开

1. Function description

    This test example demonstrates the basic communication between USARTy and USARTz to realize the infrared decoding function of serial IrDA low power consumption mode.
    First, USARTy sends TxBuffer1 data to USARTz, and USARTz receives data through interrupt and stores it in RxBuffer1.
    Subsequently, the received data is compared with the sent data, and the comparison result is stored in the TransferStatus variable.
    


2. Use environment

    Software development environment: KEIL MDK-ARM Professional Version 5.26.2.0

    Hardware environment: development board N32G457QEL_EVB_V1.1, IrDA receiver, transmitter


3. Instructions for use
    
    The system clock configuration is as follows:
    -Clock source = HSE + PLL
    -System clock = 8MHz
    
    The USART configuration is as follows:
    -Baud rate = 600 baud
    -Word length = 8 data bits
    -1 stop bit
    -Parity control disabled
    -Hardware flow control disabled (RTS and CTS signals)
    -Receiver and transmitter enable
    -IrDA mode enable
    
    The USART pin connections are as follows:
    -USART1_Tx.PA9 <-------> IrDA Transmitter
    -USART2_Rx.PB5 <-------> IrDA Receiver

    
    -GPIO.PD3 <-------> 38kHz carrier

    
    Test steps and phenomena:
    -After compiling in KEIL environment, download to MCU
    -Run to view the variable TransferStatus, where PASSED means the test passed and FAILED means the test is abnormal


4. Matters needing attention
    To use PA9/PA10, disconnect the jumper cap of MCU_TX/MCU_RX on NS-LINK
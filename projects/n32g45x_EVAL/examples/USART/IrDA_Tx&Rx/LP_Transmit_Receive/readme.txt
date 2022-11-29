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
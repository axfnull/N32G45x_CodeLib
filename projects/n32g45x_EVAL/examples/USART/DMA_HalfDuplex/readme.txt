1、功能说明
    
    该测例演示了USARTy与USARTz间通过DMA实现半双工模式的基础通信。USARTy和
USARTz可以是USART1和USART2、USART3和UART4或UART6和UART7。
    首先，DMA（中断）传输TxBuffer1数据至USARTy发送数据寄存器，随后数据发送
至USARTz。USARTz接收从DMA（中断）传来的数据，存至RxBuffer2。比较收、发数据，
比较结果存入TransferStatus1变量。
    随后，DMA（查询）传输TxBuffer2数据至USARTz发送数据寄存器，随后数据发送
至USARTy。USARTy接收从DMA（查询）传来的数据，存至RxBuffer1。比较收、发数据，
比较结果存入TransferStatus2变量。   


2、使用环境

    软件开发环境：KEIL MDK-ARM Professional Version 5.26.2.0

        硬件环境：最小系统板N32G45XV-STB_V1.1


3、使用说明
    
    系统时钟配置如下：
    - 时钟源 = HSE + PLL
    - 系统时钟 = 144MHz
    
    USART配置如下：
    - 波特率 = 115200 baud
    - 字长 = 8数据位
    - 1停止位
    - 校验控制禁用
    - 硬件流控制禁用（RTS和CTS信号）
    - 发送器使能或接收器使能
    - 半双工模式使能
    - DMA发送模式和DMA接收模式使能
    
    
    USART引脚连接如下：
    - USART1_Tx.PA9    <------->   USART2_Tx.PA2
    或
    - USART3_Tx.PB10   <------->   UART4_Tx.PC10
    或
    - UART6_Tx.PE2    <------->    UART7_Tx.PC4

    
    测试步骤与现象：
    - Demo在KEIL环境下编译后，下载至MCU
    - 复位运行，依次查看变量TransferStatus1和TransferStatus2，其中，
      PASSED为测试通过，FAILED为测试异常


4、注意事项
    USART_Tx需外接上拉电阻。
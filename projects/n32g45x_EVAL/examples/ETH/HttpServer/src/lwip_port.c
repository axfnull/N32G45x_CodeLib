/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file lwip_port.c
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include <string.h>

#include "lwip/etharp.h"
#include "lwip/ip_addr.h"
#include "lwip/tcpip.h"
#include "lwip/tcp.h"
#include "lwip/dhcp.h"
#include "log.h"

#include "n32g45x.h"
#include "n32g45x_eth.h"
#include "n32g45x_rcc.h"
#include "misc.h"

#include "FreeRTOS.h"
#include "semphr.h"

enum
{
    PHY_TYPE_LAN8710 = 0,
    PHY_TYPE_DP83848 = 1,
    PHY_TYPE_KSZ8041 = 2,
    PHY_TYPE_DM9161  = 3,
};

#define PHY_TYPE      PHY_TYPE_DM9161
#define ETH_INTERFACE ETH_INTERFACE_RMII // 0:RMII, 1:MII
#define REMAP_MODE    0

#define ETH_RXBUFNB     5
#define ETH_TXBUFNB     5
#define ETH_RX_BUF_SIZE 1524
#define ETH_TX_BUF_SIZE 1524

ETH_DMADescType g_RxDesc[ETH_RXBUFNB];
ETH_DMADescType g_TxDesc[ETH_TXBUFNB];
uint8_t g_RxBuf[ETH_RX_BUF_SIZE * ETH_RXBUFNB];
uint8_t g_TxBuf[ETH_TX_BUF_SIZE * ETH_TXBUFNB];

const uint8_t mac_address[] = {0x20, 0x34, 0x56, 0x78, 0x9a, 0xbc};

/* Phy Initial-----------------------------------------------------------------*/
uint32_t InitPHY(ETH_InitType* ETH_InitStruct)
{
    unsigned short RegValue = 0;
    uint32_t timeout;
    unsigned short PHY_Address;
    uint16_t phy_status_reg, phy_speed_bit, phy_duplex_bit;

    // search PHY address
    for (PHY_Address = 0; PHY_Address < 32; ++PHY_Address)
    {
        uint16_t val = ETH_ReadPhyRegister(PHY_Address, 3); // PHY ID register
        if (val != 0x0000 && val != 0xFFFF)
        {
            log_debug("PHY addr: %d\n", PHY_Address);
            break;
        }
    }
    if (PHY_Address > 31)
    {
        log_error("cannot comm with PHY!\n");
        while (1);
    }

    // SetReset
    if (!(ETH_WritePhyRegister(PHY_Address, PHY_BCR, (PHY_RESET|PHY_AUTONEGO))))
    {
        return ETH_ERROR;
    }
    ETH_ReadPhyRegister(PHY_Address, PHY_BCR);

    switch (PHY_TYPE)
    {
    case PHY_TYPE_LAN8710:
        phy_status_reg = 0x1f;
        phy_speed_bit  = 0x0004;
        phy_duplex_bit = 0x0010;
        break;
    case PHY_TYPE_DP83848:
        phy_status_reg = 0x10;
        phy_speed_bit  = 0x0002;
        phy_duplex_bit = 0x0004;
        // we can configure RMII mode through PHY register for DP83848
        if (ETH_INTERFACE == ETH_INTERFACE_RMII)
        {
            ETH_WritePhyRegister(PHY_Address, 0x17, 0x21);
        }
        break;
    case PHY_TYPE_KSZ8041:
        phy_status_reg = 0x1f;
        phy_speed_bit  = 0x0004;
        phy_duplex_bit = 0x0010;
        break;
    case PHY_TYPE_DM9161:
        phy_status_reg = 0x11;
        phy_speed_bit  = 0x00;
        phy_duplex_bit = 0x00;
        break;
    default:
        while (1)
            ;
    }

    if (ETH_InitStruct->AutoNegotiation != ETH_AUTONEG_DISABLE)
    {
#if 1
        // We wait for linked satus
        timeout = 0;
        do
        {
            timeout++;
        } while (!(ETH_ReadPhyRegister(PHY_Address, PHY_BSR) & PHY_LINKED_STATUS) && (timeout < PHY_READ_TO));
#endif
        if (!(ETH_WritePhyRegister(PHY_Address, PHY_BCR, PHY_AUTONEGO)))
        {
            return ETH_ERROR;
        }
        ETH_ReadPhyRegister(PHY_Address, PHY_BCR);
        // Enable Auto-Negotiation
        timeout = 0;
        do
        {
            timeout++;
        } while (!(ETH_ReadPhyRegister(PHY_Address, PHY_BSR) & PHY_AUTONEGO_COMPLETE)
                 && (timeout < (uint32_t)PHY_READ_TO));

        // Read Phy Status that is the result of the autonegotiation//786d
        RegValue = ETH_ReadPhyRegister(PHY_Address, phy_status_reg);
        if (PHY_TYPE != PHY_TYPE_DM9161)
        {
            if ((RegValue & phy_duplex_bit) != (uint32_t)RESET)
            {
                ETH_InitStruct->DuplexMode = ETH_DUPLEX_MODE_FULL;
            }
            else
            {
                ETH_InitStruct->DuplexMode = ETH_DUPLEX_MODE_HALF;
            }

            if (RegValue & phy_speed_bit)
            {
                ETH_InitStruct->SpeedMode = ETH_SPEED_MODE_10M;
            }

            else
            {
                ETH_InitStruct->SpeedMode = ETH_SPEED_MODE_100M;
            }
        }
        else
        {
            if (RegValue & 0x8000) // bit15
            {
                ETH_InitStruct->DuplexMode = ETH_DUPLEX_MODE_FULL;
                ETH_InitStruct->SpeedMode  = ETH_SPEED_MODE_100M;
            }
            else if (RegValue & 0x4000) // bit14
            {
                ETH_InitStruct->DuplexMode = ETH_DUPLEX_MODE_HALF;
                ETH_InitStruct->SpeedMode  = ETH_SPEED_MODE_100M;
            }
            else if (RegValue & 0x2000) // bit13
            {
                ETH_InitStruct->DuplexMode = ETH_DUPLEX_MODE_FULL;
                ETH_InitStruct->SpeedMode  = ETH_SPEED_MODE_10M;
            }
            else if (RegValue & 0x1000) // bit12
            {
                ETH_InitStruct->DuplexMode = ETH_DUPLEX_MODE_HALF;
                ETH_InitStruct->SpeedMode  = ETH_SPEED_MODE_10M;
            }
            else
            {
                while (1)
                    ;
            }
        }
    }
    else
    {
        RegValue = 0;
        if (ETH_InitStruct->DuplexMode == ETH_DUPLEX_MODE_FULL)
        {
            RegValue |= 0x100;
        }
        if (ETH_InitStruct->SpeedMode == ETH_SPEED_MODE_100M)
        {
            RegValue |= 0x2000;
        }
        if (!ETH_WritePhyRegister(PHY_Address, PHY_BCR, RegValue))
        {
            return ETH_ERROR;
        }

        // We wait for linked satus
        timeout = 0;
        do
        {
            timeout++;
        } while (!(ETH_ReadPhyRegister(PHY_Address, PHY_BSR) & PHY_LINKED_STATUS) && (timeout < PHY_READ_TO));
    }

    return ETH_SUCCESS;
}

static uint8_t ETH_MACDMA_Config(void)
{
    log_func();
    uint8_t rval;
    ETH_InitType ETH_InitStructure;

    ETH_DeInit();        // Reset AHB ETH clock
    ETH_SoftwareReset(); // software reset ETH module
    while (ETH_GetSoftwareResetStatus() == SET)
        ; // wait for reset over
    ETH_InitStruct(&ETH_InitStructure);

    // set ETH MAC parameters
    ETH_InitStructure.AutoNegotiation          = ETH_AUTONEG_ENABLE;
    ETH_InitStructure.LoopbackMode             = ETH_LOOPBACK_MODE_DISABLE;
    ETH_InitStructure.RetryTransmission        = ETH_RETRY_TRANSMISSION_DISABLE;
    ETH_InitStructure.AutomaticPadCRCStrip     = ETH_AUTO_PAD_CRC_STRIP_DISABLE;
    ETH_InitStructure.RxAll                    = ETH_RX_ALL_DISABLE;
    ETH_InitStructure.BroadcastFramesReception = ETH_BROADCAST_FRAMES_RECEPTION_ENABLE;
    ETH_InitStructure.PromiscuousMode          = ETH_PROMISCUOUS_MODE_DISABLE;
    ETH_InitStructure.MulticastFramesFilter    = ETH_MULTICAST_FRAMES_FILTER_NONE;
    ETH_InitStructure.UnicastFramesFilter      = ETH_UNICAST_FRAMES_FILTER_PERFECT;
#ifdef CHECKSUM_BY_HARDWARE
    ETH_InitStructure.ChecksumOffload = ETH_CHECKSUM_OFFLOAD_ENABLE;
#endif
    ETH_InitStructure.DropTCPIPChecksumErrorFrame = ETH_DROP_TCPIP_CHECKSUM_ERROR_FRAME_ENABLE;
    ETH_InitStructure.RxStoreForward              = ETH_RX_STORE_FORWARD_ENABLE;
    ETH_InitStructure.TxStoreForward              = ETH_TX_STORE_FORWARD_ENABLE;

    ETH_InitStructure.ForwardErrorFrames          = ETH_FORWARD_ERROR_FRAMES_DISABLE;
    ETH_InitStructure.ForwardUndersizedGoodFrames = ETH_FORWARD_UNDERSIZED_GOOD_FRAMES_DISABLE;
    ETH_InitStructure.SecondFrameOperate          = ETH_SECOND_FRAME_OPERATE_ENABLE;
    ETH_InitStructure.AddrAlignedBeats            = ETH_ADDR_ALIGNED_BEATS_ENABLE;
    ETH_InitStructure.FixedBurst                  = ETH_FIXED_BURST_ENABLE;
    ETH_InitStructure.RxDMABurstLen               = ETH_RX_DMA_BURST_LEN_32BEAT;
    ETH_InitStructure.TxDMABurstLen               = ETH_TX_DMA_BURST_LEN_32BEAT;
    ETH_InitStructure.DMAArbitration              = ETH_DMA_ARBITRATION_ROUND_ROBIN_RXTX_2_1;

    rval = ETH_Init(&ETH_InitStructure, InitPHY);

    if (rval) // init succeed
    {
        ETH_EnableDmaInt(ETH_DMA_INT_NIS | ETH_DMA_INT_RX, ENABLE); // Enable rx interrupt
    }

    ETH_SetMacAddr(ETH_MAC_ADDR0, (uint8_t*)mac_address);
    ETH_ConfigDmaTxDescInChainMode(g_TxDesc, g_TxBuf, ETH_TX_BUF_SIZE, ETH_TXBUFNB);
    ETH_ConfigDmaRxDescInChainMode(g_RxDesc, g_RxBuf, ETH_TX_BUF_SIZE, ETH_TXBUFNB);

#ifdef CHECKSUM_BY_HARDWARE
    for (int i = 0; i < ETH_TXBUFNB; i++)
    {
        ETH_ConfigDmaTxDescChecksumInsertion(&g_TxDesc[i], ETH_DMA_TX_DESC_CHECKSUM_TCPUDPICMP_FULL);
    }
#endif

    return !rval;
}

extern __IO ETH_DMADescType* DMATxDescToSet;
extern __IO ETH_DMADescType* DMARxDescToGet;

static uint8_t s_Buffer[ETH_TX_BUF_SIZE];
// low level function to receive packets
static struct pbuf* low_level_input(struct netif* netif)
{
    struct pbuf *p, *q;
    uint32_t len;
    int l = 0;
    uint8_t* buffer;
    p   = NULL;
    len = ETH_GetRxPacketSize();
    if (len == 0)
    {
        return NULL;
    }
    if (ETH_ERROR == ETH_RxPacket(s_Buffer, /*checkErr=*/1))
    {
        return NULL;
    }

    buffer = (uint8_t*)s_Buffer;
    p      = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
    if (p != NULL)
    {
        for (q = p; q != NULL; q = q->next)
        {
            memcpy((u8_t*)q->payload, (u8_t*)&buffer[l], q->len);
            l = l + q->len;
        }
    }
#if NETIF_DEBUG
    log_debug("recv packet, len: %d, data: ", l);
    dumphex(buffer, l);
    log_debug("\r\n");
#endif

    return p;
}

// low level function to send a packet
static err_t low_level_output(struct netif* netif, struct pbuf* p)
{
    uint32_t res;
    struct pbuf* q;
    int len         = 0;
    uint8_t* buffer = (uint8_t*)s_Buffer;

    for (q = p; q != NULL; q = q->next)
    {
        memcpy((uint8_t*)&buffer[len], q->payload, q->len);
        len += q->len;
    }

#if NETIF_DEBUG
    log_debug("send packet, len: %d, data: ", l);
    dumphex(buffer, l);
    log_debug("\n");
#endif

    res = ETH_TxPacket(buffer, len);
    if (res == ETH_ERROR)
    {
        log_error("tx pkt failed\n");
        return ERR_IF; // error status
    }
    return ERR_OK;
}

static err_t init_eth(struct netif* netif_dev)
{
    log_func();

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO | RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_GPIOC
                                | RCC_APB2_PERIPH_GPIOD | RCC_APB2_PERIPH_GPIOE | RCC_APB2_PERIPH_GPIOF
                                | RCC_APB2_PERIPH_GPIOG,
                            ENABLE);

    // MII/RMII should be configured before enabling ETH clock
    GPIO_ETH_ConfigMediaInterface(ETH_INTERFACE == ETH_INTERFACE_MII ? GPIO_ETH_MII_CFG : GPIO_ETH_RMII_CFG);

    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ETHMAC, ENABLE);

    // ETH->MACPMTCTRLSTS |= 0x80000000;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
    /* Enable the Ethernet global Interrupt */
    NVIC_InitType NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = ETH_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    ETH_ConfigGpio(ETH_INTERFACE, REMAP_MODE);

    ETH_MACDMA_Config();

#if LWIP_NETIF_HOSTNAME // LWIP_NETIF_HOSTNAME
    netif_dev->hostname = "n32g45x_lwip";
#endif
    netif_dev->name[0]    = 'n';
    netif_dev->name[1]    = 's';
    netif_dev->output     = etharp_output;    // send packet function for IP layer
    netif_dev->linkoutput = low_level_output; // send packet function for ARP
    memcpy(netif_dev->hwaddr, mac_address, 6);
    netif_dev->hwaddr_len = 6;
    netif_dev->mtu        = 1500; // the max transmit unit, 1500 for ethernet
    netif_dev->flags      = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP | NETIF_FLAG_IGMP;

    netif_set_default(netif_dev);
    netif_set_up(netif_dev);
    netif_set_link_up(netif_dev);
    return ERR_OK;
}

struct netif netif_dev;

void recv_task(void* p);
void dhcp_task(void* p);

TaskHandle_t recv_task_handle;

uint8_t lwip_port_init(void)
{
    log_func();

    tcpip_init(NULL, NULL);

    ip_addr_t ip, msk, gw;
#if LWIP_DHCP
    ip.addr  = 0;
    msk.addr = 0;
    gw.addr  = 0;
#else  // !LWIP_DHCP
    IP4_ADDR(&ip, 10, 70, 54, 39);
    IP4_ADDR(&msk, 255, 255, 255, 0);
    IP4_ADDR(&gw, 10, 70, 54, 10);
#endif // LWIP_DHCP

    netif_add(&netif_dev, &ip, &msk, &gw, NULL, init_eth, tcpip_input);

    xTaskCreate(recv_task, "recv_task", DEFAULT_THREAD_STACKSIZE, NULL, TCPIP_THREAD_PRIO + 1, &recv_task_handle);

#if LWIP_DHCP
    sys_thread_new("dhcp_task", dhcp_task, NULL, DEFAULT_THREAD_STACKSIZE * 2, TCPIP_THREAD_PRIO - 1);
#endif

    ETH_EnableTxRx();

    return 0;
}

void recv_task(void* p)
{
    log_func();
    while (1)
    {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) // packet received event
        {
            err_t err;
            struct pbuf* p;
            while (1)
            {
                p = low_level_input(&netif_dev); // call low_level_input to receive
                if (p == NULL)
                {
                    break;
                }
                err = netif_dev.input(p, &netif_dev); // process packet
                if (err != ERR_OK)
                {
                    LWIP_DEBUGF(NETIF_DEBUG, ("IP input error\n"));
                    pbuf_free(p);
                    p = NULL;
                    // break;
                }
            }
        }
    }
}

void ETH_IRQHandler(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* Frame received */
    if (ETH_GetDmaFlagStatus(ETH_DMA_FLAG_RX) == SET)
    {
        /* Give the semaphore to wakeup LwIP task */
        vTaskNotifyGiveFromISR(recv_task_handle, &xHigherPriorityTaskWoken);
    }

    /* Clear the interrupt flags. */
    /* Clear the Eth DMA Rx IT pending bits */
    ETH_ClrDmaIntPendingBit(ETH_DMA_INT_RX);
    ETH_ClrDmaIntPendingBit(ETH_DMA_INT_NIS);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#if LWIP_DHCP
void dhcp_task(void* p)
{
    log_func();

    dhcp_start(&netif_dev);

    while (1)
    {
        if (netif_dev.ip_addr.addr != 0)
        {
            break;
        }
        vTaskDelay(25);
    }

    uint32_t ip   = netif_dev.ip_addr.addr;
    uint32_t mask = netif_dev.netmask.addr;
    uint32_t gw   = netif_dev.gw.addr;

    log_info("DHCP over\n");
    log_info("IP:       %d.%d.%d.%d\n", ((uint8_t*)&ip)[0], ((uint8_t*)&ip)[1], ((uint8_t*)&ip)[2], ((uint8_t*)&ip)[3]);
    log_info("Mask:     %d.%d.%d.%d\n",
             ((uint8_t*)&mask)[0],
             ((uint8_t*)&mask)[1],
             ((uint8_t*)&mask)[2],
             ((uint8_t*)&mask)[3]);
    log_info("Gateway:  %d.%d.%d.%d\n", ((uint8_t*)&gw)[0], ((uint8_t*)&gw)[1], ((uint8_t*)&gw)[2], ((uint8_t*)&gw)[3]);

    // delete this task
    vTaskDelete(NULL);
    while (1)
        ;
}
#endif // LWIP_DHCP

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
 * @file lwipopts.h
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#define LWIP_IPV4 1
#define LWIP_IPV6 0

#define LWIP_SOCKET  (NO_SYS == 0)
#define LWIP_NETCONN (NO_SYS == 0)

#define LWIP_IGMP LWIP_IPV4
#define LWIP_ICMP LWIP_IPV4

#define LWIP_SNMP LWIP_UDP
// #define MIB2_STATS                 LWIP_SNMP

#define LWIP_DNS            LWIP_UDP
#define LWIP_MDNS_RESPONDER LWIP_UDP

#define LWIP_NUM_NETIF_CLIENT_DATA (LWIP_MDNS_RESPONDER)

#define SO_REUSE    1
#define LWIP_TCP_KEEPALIVE      1
#define LWIP_HAVE_LOOPIF        1
#define LWIP_NETIF_LOOPBACK     1
#define LWIP_LOOPBACK_MAX_PBUFS 10

#define TCP_LISTEN_BACKLOG 1

#define LWIP_COMPAT_SOCKETS 1
#define LWIP_SO_RCVTIMEO    1
#define LWIP_SO_RCVBUF      1

#define LWIP_TCPIP_CORE_LOCKING 1

#define LWIP_NETIF_LINK_CALLBACK   1
#define LWIP_NETIF_STATUS_CALLBACK 1

#define IP_FRAG               1
#define IP_REASSEMBLY         1
#define IP_SOF_BROADCAST      1
#define IP_SOF_BROADCAST_RECV 1

#define LWIP_DEBUG 1

#ifdef LWIP_DEBUG

#define LWIP_DBG_MIN_LEVEL 0
#define PPP_DEBUG          LWIP_DBG_OFF
#define MEM_DEBUG          LWIP_DBG_OFF
#define MEMP_DEBUG         LWIP_DBG_OFF
#define PBUF_DEBUG         LWIP_DBG_OFF
#define API_LIB_DEBUG      LWIP_DBG_OFF
#define API_MSG_DEBUG      LWIP_DBG_OFF
#define TCPIP_DEBUG        LWIP_DBG_OFF
#define NETIF_DEBUG        LWIP_DBG_OFF
#define SOCKETS_DEBUG      LWIP_DBG_OFF
#define DNS_DEBUG          LWIP_DBG_OFF
#define AUTOIP_DEBUG       LWIP_DBG_OFF
#define DHCP_DEBUG         LWIP_DBG_OFF
#define IP_DEBUG           LWIP_DBG_OFF
#define IP_REASS_DEBUG     LWIP_DBG_OFF
#define ICMP_DEBUG         LWIP_DBG_OFF
#define IGMP_DEBUG         LWIP_DBG_OFF
#define UDP_DEBUG          LWIP_DBG_OFF
#define TCP_DEBUG          LWIP_DBG_OFF
#define TCP_INPUT_DEBUG    LWIP_DBG_OFF
#define TCP_OUTPUT_DEBUG   LWIP_DBG_OFF
#define TCP_RTO_DEBUG      LWIP_DBG_OFF
#define TCP_CWND_DEBUG     LWIP_DBG_OFF
#define TCP_WND_DEBUG      LWIP_DBG_OFF
#define TCP_FR_DEBUG       LWIP_DBG_OFF
#define TCP_QLEN_DEBUG     LWIP_DBG_OFF
#define TCP_RST_DEBUG      LWIP_DBG_OFF
#endif

#define LWIP_DBG_TYPES_ON (LWIP_DBG_ON | LWIP_DBG_TRACE | LWIP_DBG_STATE | LWIP_DBG_FRESH | LWIP_DBG_HALT)

#define SYS_LIGHTWEIGHT_PROT 1

// NO_SYS==1: do not use OS
#define NO_SYS 0

#define MEM_ALIGNMENT 4

#define MEM_SIZE (54 * 1024) // heap size for lwip, set a big value if transmit many data

/* ---------- TCP options ---------- */
#define LWIP_TCP 1 // use TCP
#define TCP_TTL  128

// max segment size of TCP
#define TCP_MSS (1500 - 40) // TCP_MSS = (MTU - IP head size - TCP head size)

/* ---------- DHCP options ---------- */
#define LWIP_DHCP 1

/* ---------- UDP options ---------- */
#define LWIP_UDP 1
#define UDP_TTL  128

/* ---------- Statistics options ---------- */
#define LWIP_STATS         0
#define LWIP_PROVIDE_ERRNO 1

#define CHECKSUM_BY_HARDWARE // enable hardware checksum

#ifdef CHECKSUM_BY_HARDWARE
#define CHECKSUM_GEN_ICMP    0
#define CHECKSUM_CHECK_ICMP  0
#define CHECKSUM_GEN_ICMP6   0
#define CHECKSUM_CHECK_ICMP6 0
#define CHECKSUM_GEN_IP      0
#define CHECKSUM_GEN_UDP     0
#define CHECKSUM_GEN_TCP     0
#define CHECKSUM_CHECK_IP    0
#define CHECKSUM_CHECK_UDP   0
#define CHECKSUM_CHECK_TCP   0
#else
#define CHECKSUM_GEN_ICMP    1
#define CHECKSUM_CHECK_ICMP  1
#define CHECKSUM_GEN_ICMP6   1
#define CHECKSUM_CHECK_ICMP6 1
#define CHECKSUM_GEN_IP      1
#define CHECKSUM_GEN_UDP     1
#define CHECKSUM_GEN_TCP     1
#define CHECKSUM_CHECK_IP    1
#define CHECKSUM_CHECK_UDP   1
#define CHECKSUM_CHECK_TCP   1
#endif

#define LWIP_COMPAT_MUTEX 1
#define LWIP_COMPAT_MUTEX_ALLOWED
#define LWIP_SKIP_CONST_CHECK

#include <stdlib.h>
#define LWIP_RAND rand

#define TCPIP_THREAD_NAME         "TCP/IP"
#define TCPIP_THREAD_STACKSIZE    512
#define TCPIP_MBOX_SIZE           50
#define DEFAULT_UDP_RECVMBOX_SIZE 20
#define DEFAULT_TCP_RECVMBOX_SIZE 20
#define DEFAULT_ACCEPTMBOX_SIZE   20
#define DEFAULT_THREAD_STACKSIZE  256
#define TCPIP_THREAD_PRIO         (3)

#define LWIP_PLATFORM_ASSERT(x)                                                                                        \
    do                                                                                                                 \
    {                                                                                                                  \
        printf("Assertion \"%s\" failed at line %d in %s\r\n", x, __LINE__, __FILE__);                                 \
    } while (0)

#endif /* __LWIPOPTS_H__ */

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
 * @file test_tcp_server.c
 * @brief This is a simple example for tcp server.
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "lwip/sockets.h"
#include "lwip/tcpip.h"
#include "stdlib.h"
#include "stdio.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"

#define MAX_CONNECT MEMP_NUM_NETCONN

extern int errno;

typedef struct{
    int sock;
    struct sockaddr_in client;
}Connect_Host_t;

Connect_Host_t connect_host[MAX_CONNECT];

char recv_buff[1024];/*!<receive buffer*/
char send_buff[1024];/*!<send buffer*/

extern struct netif netif_dev;/*it's defined lwip_port.c*/
/**
 * @brief  accept a client.
 * @param  point to task parameters.
 * @param None.
 */
void test_tcp_accept_task(void *p)
{
    int backlog = 2;
    int so_keepalive = 1;
    int so_reuseaddr = 1;
//  int so_reuseport = 1;
    struct sockaddr_in serverAddr;
    struct sockaddr_in clientAddr;
    int sockServer =  socket(AF_INET,SOCK_STREAM, 0);
    while (1)
    {
        if (netif_dev.ip_addr.addr != 0)
        {
            break;
        }
        vTaskDelay(25);
    }
    /*set tcp server ip address*/
    socklen_t socklen = sizeof(clientAddr);
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = netif_dev.ip_addr.addr;
    serverAddr.sin_port = htons(12345);
    log_info("TCP SERVER \r\nIP:%s\r\nPORT:%d\r\n",ipaddr_ntoa(&netif_dev.ip_addr),12345);
    if(bind(sockServer,(struct sockaddr *)&serverAddr,sizeof(struct sockaddr))< 0){
        log_info("error in bind\r\n");
    }
    /*reuse port*/
    setsockopt(sockServer, IPPROTO_TCP,SO_KEEPALIVE,&so_keepalive,sizeof(int));
    setsockopt(sockServer, SOL_SOCKET,SO_REUSEADDR,&so_reuseaddr,sizeof(int));
    listen(sockServer,backlog);
    for(;;)
    {
        /*accept client connection*/
        int sockClient = accept(sockServer,(struct sockaddr *)&clientAddr,&socklen);
        if(sockClient < 0)
        {
            continue;
        }
        for(int i = 0; i< MAX_CONNECT; i++)
        {
            if(connect_host[i].sock == -1)
            {
                /*store a client information*/
                connect_host[i].sock = sockClient;
                memcpy(&connect_host[i].client,&clientAddr,sizeof(struct sockaddr_in));
                break;
            }
        }
    }
}

/**
 * @brief  Send infomation to client.
 * @param  point to task parameters.
 * @param None.
 */
void test_tcp_respone_task(void *p)
{
    int max_fd = 0;
    struct timeval timeout;
    fd_set input_fd;
    
    while (1)
    {
        /*check ip address*/
        if (netif_dev.ip_addr.addr != 0)
        {
            break;
        }
        vTaskDelay(25);
    }
    /*initialize fd*/
    FD_ZERO(&input_fd);
    /*set timeout*/
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;
    for(;;)
    {
        max_fd = -1;
        FD_ZERO(&input_fd);
        for(int i= 0; i< MAX_CONNECT; i++)
        {
            if(connect_host[i].sock != -1)
            {
                FD_SET(connect_host[i].sock,&input_fd);
                if(connect_host[i].sock > max_fd)
                    max_fd = connect_host[i].sock;
            }
        }
        /*refer to network programming*/
        if(select((max_fd+1),&input_fd, NULL,NULL,&timeout) < 1)
        {
            continue;
        }
        for(int i= 0; i< MAX_CONNECT; i++)
        {
            int sock = connect_host[i].sock;
            if(sock!= -1)
            {
                /*check fd structure*/
                if(FD_ISSET(sock,&input_fd))
                {
                    memset(recv_buff,0,1024);
                    memset(send_buff,0,1024);
                    /*receive a message from client*/
                    int len = recv(sock,recv_buff,1024,MSG_DONTWAIT);
                    if(len > 0)
                    {
                    /*send massage to client*/
                        len = snprintf(send_buff,1024,"Client IP:%s\r\nClient PORT:%d\r\nReceive Message:%s\r\n",\
                                                ipaddr_ntoa((void *)&connect_host[i].client.sin_addr.s_addr),\
                                                connect_host[i].client.sin_port,\
                                                recv_buff);
                        send(sock,send_buff,len,0);
                    }
                    else if(len == 0 && errno == ENOTCONN)
                    {
                        
                        close(connect_host[i].sock);
                        connect_host[i].sock = -1;
                        
                    }
                    else
                    {
                        close(connect_host[i].sock);
                        connect_host[i].sock = -1;
                    }
                }
            }
        }
    }
}
/**
 * @brief  Initialize tcp task.
 * @param None.
 * @param None.
 */
void test_tcp_init(void)
{
    for(int i = 0 ;i < MAX_CONNECT; i++)
    {
        connect_host[i].sock = -1;
    }
    sys_thread_new("tcp_respone", test_tcp_respone_task,NULL,512, TCPIP_THREAD_PRIO-3);
    sys_thread_new("tcp_accept", test_tcp_accept_task,NULL,512, TCPIP_THREAD_PRIO-4);
    
}

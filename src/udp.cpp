// BSD 3-Clause License
//
// Copyright (c) 2020, TIANBOT
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "udp.h"

void *Udp::udpRecv(void *p)
{
    uint8_t recv_buff[UDP_RECV_BUFF_LEN];
    int recv_len = 0;
    Udp *p_this = (Udp *)p;
    struct sockaddr_in src_addr = {0};
    socklen_t len = sizeof(src_addr);

    while (p_this->running_)
    {
        memset(recv_buff, 0, UDP_RECV_BUFF_LEN);

        if ((recv_len = recvfrom(p_this->socket_fd_, recv_buff, UDP_RECV_BUFF_LEN, 0, (struct sockaddr *)&src_addr, &len)) < 0)
        {
            continue;
        }
        if (p_this->dataCallback_ != NULL)
        {
            p_this->dataCallback_(recv_buff, recv_len);
        }
    }
    return NULL;
}

Udp::Udp()
{
    socket_fd_ = -1;
    bzero(&server_addr_, sizeof(server_addr_));
    bzero(&client_addr_, sizeof(client_addr_));

    client_addr_length_ = sizeof(client_addr_);
    server_addr_length_ = sizeof(server_addr_);

    running_ = false;
    dataCallback_ = NULL;
    recv_thread_ = 0;
}

bool Udp::init(const char *client_ip, int client_port, int server_port, udp_recv_cb_t cb)
{
    int ret;
    pthread_attr_t attr;

    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0)
    {
        perror("socket() error");
        return false;
    }

    int on = 1;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(int)) < 0)
    {
        perror("setsockopt() error ");
        ::close(socket_fd_);
        return false;
    }

    server_addr_.sin_family = AF_INET;
    server_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr_.sin_port = htons(server_port);

    client_addr_.sin_family = AF_INET;
    client_addr_.sin_addr.s_addr = inet_addr(client_ip);
    client_addr_.sin_port = htons(client_port);

    if (bind(socket_fd_, (struct sockaddr *)&server_addr_, sizeof(server_addr_)) < 0)
    {
        perror("bind() error");
        ::close(socket_fd_);
        return false;
    }

    running_ = 1;

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    ret = pthread_create(&recv_thread_, &attr, udpRecv, this);
    if (0 != ret)
    {
        perror("pthread_create() error");
        ::close(socket_fd_);
        return false;
    }
    dataCallback_ = cb;
    return true;
}

bool Udp::send(uint8_t *buff, uint16_t len)
{
    int ret = 0;
    int sended_len = 0;

    if (socket_fd_ <= 0)
    {
        return false;
    }

    //反复发送，直到全部发完
    while (sended_len != len)
    {
        int retlen = 0;
        retlen = sendto(socket_fd_, buff + sended_len, len - sended_len, 0, (struct sockaddr *)&client_addr_, client_addr_length_);
        if (retlen < 0)
        {
            return false;
        }
        sended_len += retlen;
    }
    return true;
}

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

#include "iexxx.h"

void *IExxx::lidarRecv(void *p)
{
    uint8_t recv_buff[LIDAR_RECV_BUFF_SIZE];
    int recv_len = 0;
    IExxx *p_this = (IExxx *)p;
    while (p_this->running_)
    {
        memset(recv_buff, 0, LIDAR_RECV_BUFF_SIZE);

        //if ((recv_len = read(p_this->fd_, recv_buff, sizeof(LIDAR_RECV_BUFF_SIZE))) == -1)
        //{
        //    continue;
        //}
        p_this->dataCallback(recv_buff, recv_len);
    }
    return NULL;
}

void IExxx::dataCallback(uint8_t *buff, int len)
{
    
}

IExxx::IExxx(ros::NodeHandle *nh):OsightLidar(nh)
{
    nh_.param<std::string>("lidar_ip", lidar_ip_, DEFAULT_LIDAR_IP);
    nh_.param<int>("lidar_port", lidar_port_, DEFAULT_LIDAR_PORT);
    nh_.param<int>("host_port", host_port_, DEFAULT_HOST_PORT);

    socket_fd_ = -1;
    bzero(&host_addr_, sizeof(host_addr_));
    bzero(&lidar_addr_, sizeof(lidar_addr_));

    lidar_addr_length_ = sizeof(lidar_addr_);
    host_addr_length_ = sizeof(host_addr_);

    running_ = 0;
}

bool IExxx::init(void)
{
    int ret;
    pthread_attr_t attr;

    recv_thread_ = 0;

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

    host_addr_.sin_family = AF_INET;
    host_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
    host_addr_.sin_port = htons(host_port_);

    if (bind(socket_fd_, (struct sockaddr *)&host_addr_, sizeof(host_addr_)) < 0)
    {
        perror("bind() error");
        ::close(socket_fd_);
        return false;
    }

    running_ = 1;

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    ret = pthread_create(&recv_thread_, &attr, lidarRecv, this);
    if (0 != ret)
    {
        perror("pthread_create() error");
        ::close(socket_fd_);
        return false;
    }

    return true;
}

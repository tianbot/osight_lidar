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

#include "osight_lidar.h"

OsightLidar::OsightLidar(ros::NodeHandle *nh) : nh_(*nh)
{
    seq_ = 0;
    nh_.param<std::string>("frame_id", frame_id_, DEFAULT_FRAME_ID);
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1);
}

void OsightLidar::lidarDataCallback(vector<float> ranges, vector<float> intensities, struct LidarParam lidar_param)
{
    sensor_msgs::LaserScan scan_msg;
    scan_msg.header.seq = seq_++;
    scan_msg.header.stamp = ros::Time::now();
    scan_msg.header.frame_id = frame_id_;
    scan_msg.angle_min = lidar_param.angle_min;
    scan_msg.angle_max = lidar_param.angle_max;
    scan_msg.angle_increment = lidar_param.angle_increment;
    scan_msg.time_increment = lidar_param.time_increment;
    scan_msg.scan_time = lidar_param.scan_time;
    scan_msg.range_min = lidar_param.range_min;
    scan_msg.range_max = lidar_param.range_max;
    scan_msg.ranges = ranges;
    scan_msg.intensities = intensities;
    scan_pub_.publish(scan_msg);
}

void OsightLidar::invertUint8(uint8_t *dest_buf, uint8_t *src_buf)
{
    int i;
    uint8_t tmp;
    tmp = 0;
    for (i = 0; i < 8; i++)
    {
        if (*src_buf & (1 << i))
        {
            tmp |= 1 << (7 - i);
        }
    }
    *dest_buf = tmp;
}
void OsightLidar::invertUint16(uint16_t *dest_buf, uint16_t *src_buf)
{
    int i;
    uint16_t tmp;
    tmp = 0;
    for (i = 0; i < 16; i++)
    {
        if (*src_buf & (1 << i))
        {
            tmp |= 1 << (15 - i);
        }
    }
    *dest_buf = tmp;
}

uint16_t OsightLidar::crc16(uint8_t *buff, uint32_t len)
{
    uint16_t wCRCin = 0xFFFF;
    uint16_t wCPoly = 0x8005;
    uint8_t wChar = 0;
    int i;

    while (len--)
    {
        wChar = *(buff++);
        invertUint8(&wChar, &wChar);
        wCRCin = wCRCin ^ (wChar << 8);
        for (i = 0; i < 8; i++)
        {
            if (wCRCin & 0x8000)
            {
                wCRCin = (wCRCin << 1) ^ wCPoly;
            }
            else
            {
                wCRCin = wCRCin << 1;
            }
        }
    }
    invertUint16(&wCRCin, &wCRCin);
    //wCRCin = HTONS(wCRCin);

    return wCRCin;
}

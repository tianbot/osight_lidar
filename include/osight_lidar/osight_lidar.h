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

#ifndef _OSIGHT_LIDAR_H_
#define _OSIGHT_LIDAR_H_

#include "ros/ros.h"
#include "stdint.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>

#define DEFAULT_LIDAR_MODEL "iexxx"
#define DEFAULT_FRAME_ID "laser"

#define DEFAULT_LIDAR_IP "192.168.1.10"
#define DEFAULT_LIDAR_PORT 6500
#define DEFAULT_HOST_PORT 5500

using namespace std;

struct LidarParam
{
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
};

class OsightLidar
{
public:
    OsightLidar(ros::NodeHandle *nh);
    //~OsightLidar();
    virtual bool init(void) = 0;
    virtual void updateParam(void) = 0;

protected:
    ros::NodeHandle nh_;
    void lidarDataCallback(vector<float> ranges, vector<float> intensities, struct LidarParam lidar_param);
    uint16_t crc16(uint8_t *buff, uint32_t len);
    std::string lidar_ip_;
    int host_port_;
    int lidar_port_;
    struct LidarParam lidar_param_;

private:
    uint32_t seq_;
    std::string frame_id_;
    ros::Publisher scan_pub_;
    void invertUint8(uint8_t *dest_buf, uint8_t *src_buf);
    void invertUint16(uint16_t *dest_buf, uint16_t *src_buf);
};

#endif

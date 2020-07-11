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

#include "ros/ros.h"
#include "iexxx.h"

int main(int argc, char *argv[])
{
    OsightLidar *lidar;

    std::string lidar_model;

    ros::init(argc, argv, "osight_lidar_node");

    ros::NodeHandle nh("osight_lidar");

    nh.param<std::string>("lidar_model", lidar_model, DEFAULT_LIDAR_MODEL);

    ros::Rate loop_rate(10);

    if (lidar_model == "iexxx")
    {
        lidar = new IExxx(&nh);
    }
    else
    {
        ROS_INFO("lidar [%s] driver not implemented", lidar_model.c_str());
        exit(-1);
    }

    int count = 10;
    while (--count)
    {
        if (lidar->init())
        {
            ROS_INFO("lidar [%s] init successfully", lidar_model.c_str());
            break;
        }
        else
        {
            ROS_WARN("lidar [%s] init failed, retry ...", lidar_model.c_str());
            ros::Duration(1).sleep();
        }
    }
    if (count == 0)
    {
        ROS_ERROR("lidar [%s] init failed, exit", lidar_model.c_str());
        delete lidar;
        return -1;
    }
    ros::Duration(1).sleep();
    lidar->updateParam();
    ros::Duration(1).sleep();
    lidar->startTransferData();

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete lidar;
    return 0;
}
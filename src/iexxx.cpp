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

IExxx::IExxx(ros::NodeHandle *nh) : OsightLidar(nh)
{
    ranges.clear();
    intensities.clear();
    //lidar_param_.range_max = DEFAULT_MAX_RANGES;
    nh_.param<float>("range_max", lidar_param_.range_max, DEFAULT_MAX_RANGES);
    //lidar_param_.range_min = DEFAULT_MIN_RANGES;
    nh_.param<float>("range_min", lidar_param_.range_min, DEFAULT_MIN_RANGES);
    //lidar_param_.scan_time = DEFAULT_SCAN_TIME;
    nh_.param<float>("scan_time", lidar_param_.scan_time, DEFAULT_SCAN_TIME);
    //lidar_param_.angle_min = DEFAULT_ANGLE_MIN;
    nh_.param<float>("angle_min", lidar_param_.angle_min, DEFAULT_ANGLE_MIN);
    //lidar_param_.angle_max = DEFAULT_ANGLE_MAX;
    nh_.param<float>("angle_max", lidar_param_.angle_max, DEFAULT_ANGLE_MAX);
    //lidar_param_.time_increment = DEFAULT_TIME_INCREMENT;
    nh_.param<float>("time_increment", lidar_param_.time_increment, DEFAULT_TIME_INCREMENT);
    //lidar_param_.angle_increment = DEFAULT_ANGLE_INCREMENT;
    nh_.param<float>("angle_increment", lidar_param_.angle_increment, DEFAULT_ANGLE_INCREMENT);
    ROS_INFO("lidar default min angle: %f", lidar_param_.angle_min);
    ROS_INFO("lidar default max angle: %f", lidar_param_.angle_max);
    ROS_INFO("lidar default angle increment: %f", lidar_param_.angle_increment);
    ROS_INFO("lidar default min range: %f", lidar_param_.range_min);
    ROS_INFO("lidar default max range: %f", lidar_param_.range_max);
    ROS_INFO("lidar default scan time: %f", lidar_param_.scan_time);
    ROS_INFO("lidar default time increment: %f", lidar_param_.time_increment);
    nh_.param<std::string>("lidar_ip", lidar_ip_, DEFAULT_LIDAR_IP);
    nh_.param<int>("lidar_port", lidar_port_, DEFAULT_LIDAR_PORT);
    nh_.param<int>("host_port", host_port_, DEFAULT_HOST_PORT);
    IPConfigService_ = nh_.advertiseService<osight_lidar::IPConfig::Request, osight_lidar::IPConfig::Response>("ip_cfg", boost::bind(&IExxx::IPCfg, this, _1, _2));
}

IExxx::~IExxx()
{
    udp_->close();
    delete udp_;
}

bool IExxx::init(void)
{
    udp_ = new Udp();
    ROS_INFO("lidar IP: %s Port: %d", lidar_ip_.c_str(), lidar_port_);
    return udp_->init(lidar_ip_.c_str(), lidar_port_, host_port_, boost::bind(&IExxx::dataCallback, this, _1, _2));
}

void IExxx::updateParam(void)
{
    struct ParamSyncReq req;
    req.msg_id = htonl(PARA_SYNC_REQ);
    req.crc = crc16((uint8_t *)&req, sizeof(req) - 2);
    udp_->send((uint8_t *)&req, sizeof(req));
}

void IExxx::startTransferData(void)
{
    struct StartMeasureTransmissionReq req;
    req.msg_id = htonl(START_MEASURE_TRANSMISSION_REQ);
    req.function_id = START_DATA_TRANSFER;
    req.crc = crc16((uint8_t *)&req, sizeof(req) - 2);
    udp_->send((uint8_t *)&req, sizeof(req));
}

void IExxx::stopTransferData(void)
{
    struct StartMeasureTransmissionReq req;
    req.msg_id = htonl(START_MEASURE_TRANSMISSION_REQ);
    req.function_id = STOP_DATA_TRANSFER;
    req.crc = htons(crc16((uint8_t *)&req, sizeof(req) - 2));
    udp_->send((uint8_t *)&req, sizeof(req));
}

bool IExxx::IPCfg(osight_lidar::IPConfig::Request &request, osight_lidar::IPConfig::Response &res)
{
    struct IPConfigReq req;
    req.msg_id = htonl(SET_STATIC_IP_REQ);
    req.dev_ip = inet_addr(request.dev_ip.c_str());
    req.dev_port = inet_addr(request.dev_port.c_str());
    req.host_ip = inet_addr(request.dev_ip.c_str());
    req.host_port = inet_addr(request.dev_ip.c_str());
    req.mask = inet_addr(request.mask.c_str());
    req.gateway = inet_addr(request.gateway.c_str());

    ip_cfg_err_ = 0xFFFF;

    req.crc = htons(crc16((uint8_t *)&req, sizeof(req) - 2));
    udp_->send((uint8_t *)&req, sizeof(req));
    ros::Duration(0.5).sleep();
    res.error_no = ip_cfg_err_;
    return true;
}

void IExxx::dataCallback(uint8_t *buff, int len)
{
    uint32_t msg_id = (buff[0] << 24) + (buff[1] << 16) + (buff[2] << 8) + buff[3];
    uint16_t crc = crc16(buff, len - sizeof(uint16_t));
    if (crc != (buff[len - 1] << 8) + (buff[len - 2]))
    {
        ROS_WARN("crc error %04x, %02x, %02x\r\n", crc, buff[len - 2], buff[len - 1]);
        return;
    }
    switch (msg_id)
    {
    case PARA_SYNC_RSP:
        {
            ROS_INFO("Lidar parameter update.");
            struct ParamSyncRsp *p = (struct ParamSyncRsp *)buff;
            if (lidar_param_.angle_min != (float)((int32_t)ntohl(p->start_angle) / 1000.0 * DEG2RAD - M_PI / 2))
            {
                lidar_param_.angle_min = (int32_t)ntohl(p->start_angle) / 1000.0 * DEG2RAD - M_PI / 2;
                ROS_INFO("lidar min angle update: %f", lidar_param_.angle_min);
            }
            if (lidar_param_.angle_max != (float)(ntohl(p->stop_angle) / 1000.0 * DEG2RAD - M_PI / 2))
            {
                lidar_param_.angle_max = ntohl(p->stop_angle) / 1000.0 * DEG2RAD - M_PI / 2;
                ROS_INFO("lidar max angle update: %f", lidar_param_.angle_max);
            }
            if (lidar_param_.angle_increment != (float)(ntohl(p->angular_resolution) / 10000000.0 * DEG2RAD))
            {
                lidar_param_.angle_increment = ntohl(p->angular_resolution) / 10000000.0 * DEG2RAD;
                ROS_INFO("lidar angle increment update: %f", lidar_param_.angle_increment);
            }
            if (lidar_param_.range_max != (float)(ntohl(p->max_distance)/10000.0))
            {
                lidar_param_.range_max = ntohl(p->max_distance)/10000.0;
                ROS_INFO("lidar max range update: %f", lidar_param_.range_max);
            }
            if (lidar_param_.scan_time != (float)(1.0 / p->speed))
            {
                lidar_param_.scan_time = 1.0 / p->speed;
                ROS_INFO("lidar scan time update: %f", lidar_param_.scan_time);
            }
            if (lidar_param_.time_increment != (float)(lidar_param_.scan_time / (2 * M_PI / lidar_param_.angle_increment)))
            {
                lidar_param_.time_increment = lidar_param_.scan_time / (2 * M_PI / lidar_param_.angle_increment);
                ROS_INFO("lidar time increment update: %f", lidar_param_.time_increment);
            }
        }
        break;
    case PARA_CHANGED_IND_RSP:
        updateParam();
        break;
    case PARA_DEVICE_CONFIGURATION_RSP:
        break;
    case PARA_ALARM_CONFIGURATION_RSQ:
        break;
    case MEAS_DATA_PACKAGE_RSP:
        {
            struct MeasureDataRsp *p = (struct MeasureDataRsp *)buff;
            int i;
            uint8_t *p_data = p->data;
            uint32_t range_data;
            uint32_t intensity_data;
            p->msg_id = ntohl(p->msg_id);
            p->serial_num = ntohl(p->serial_num);
            p->product_num = ntohl(p->product_num);
            p->time_stamp = ntohl(p->time_stamp);
            p->start_angle = (int32_t)ntohl(p->start_angle);
            p->stop_angle = (int32_t)ntohl(p->stop_angle);
            p->micro_second = ntohl(p->micro_second);
            p->angular_resolution = ntohl(p->angular_resolution);
            p->vertical_angle = ntohs(p->vertical_angle);
            p->max_point_num = ntohs(p->max_point_num);
            p->current_point_num = ntohs(p->current_point_num);

            if (lidar_param_.angle_min != (float)(p->start_angle / 1000.0 * DEG2RAD - M_PI / 2))
            {
                lidar_param_.angle_min = p->start_angle / 1000.0 * DEG2RAD - M_PI / 2;
                ROS_INFO("lidar min angle update: %f", lidar_param_.angle_min);
            }
            if (lidar_param_.angle_max != (float)(p->stop_angle / 1000.0 * DEG2RAD - M_PI / 2))
            {
                lidar_param_.angle_max = p->stop_angle / 1000.0 * DEG2RAD - M_PI / 2;
                ROS_INFO("lidar max angle update: %f", lidar_param_.angle_max);
            }
            if (lidar_param_.angle_increment != (float)(p->angular_resolution / 10000000.0 * DEG2RAD))
            {
                lidar_param_.angle_increment = p->angular_resolution / 10000000.0 * DEG2RAD;
                ROS_INFO("lidar angle increment update: %f", lidar_param_.angle_increment);
            }

            if (p->pkg_num == 0)
            {
                ranges.clear();
                intensities.clear();
            }

            for (i = 0; i < p->current_point_num; i++)
            {
                UNPACK_4_BYTE(p_data, range_data);
                ranges.push_back(range_data / 10000.0);
                if (p->intensity_status == 2)
                {
                    UNPACK_2_BYTE(p_data, intensity_data);
                    intensities.push_back(intensity_data);
                }
                else if (p->intensity_status == 1)
                {
                    UNPACK_1_BYTE(p_data, intensity_data);
                    intensities.push_back(intensity_data);
                }
            }

            if (p->total_pkgs == p->pkg_num + 1)
            {
                lidarDataCallback(ranges, intensities, lidar_param_);
            }
        }
        break;
    case LOG_GET_RSP:
        break;
    case TIME_REPORT_INF:
        break;
    case ACTIVE_FILTER_RSP:
        break;
    case SET_CALIBRATION_MODE_RSP:
        break;
    case SET_NET_MODE_RSP:
        break;
    case SET_STATIC_IP_RSP:
        {
            struct IPConfigRsp *p = (struct IPConfigRsp *)buff;
            p->msg_id = ntohl(p->msg_id);
            p->error_no = ntohs(p->error_no);
            ip_cfg_err_ = p->error_no;
        }
        break;
    default:
        break;
    }
}

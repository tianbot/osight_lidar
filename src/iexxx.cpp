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

const struct ErrStr err[] = {
    {0x0000, "succeed"},
    {0x0001, "angle resolution configuration not supported"},
    {0x0002, "speed configuration not supported"},
    {0x0003, "intensity configuration not supported"},
    {0x0101, "angle resolution param error"},
    {0x0102, "speed param error"},
    {0x0103, "intensity param error"},
    {0x0107, "outlier param error"},
    {0x0108, "echo param error"},
    {0x010A, "IP param error"},
    {0xFFFF, "response timeout"}};

IExxx::IExxx(ros::NodeHandle *nh) : OsightLidar(nh)
{
    ranges.clear();
    intensities.clear();
    nh_.param<float>("range_max", lidar_param_.range_max, DEFAULT_MAX_RANGES);
    nh_.param<float>("range_min", lidar_param_.range_min, DEFAULT_MIN_RANGES);
    nh_.param<float>("scan_time", lidar_param_.scan_time, DEFAULT_SCAN_TIME);
    nh_.param<float>("angle_min", lidar_param_.angle_min, DEFAULT_ANGLE_MIN);
    nh_.param<float>("angle_max", lidar_param_.angle_max, DEFAULT_ANGLE_MAX);
    nh_.param<float>("time_increment", lidar_param_.time_increment, DEFAULT_TIME_INCREMENT);
    nh_.param<float>("angle_increment", lidar_param_.angle_increment, DEFAULT_ANGLE_INCREMENT);
    ROS_INFO("lidar default min angle: %f", lidar_param_.angle_min / DEG2RAD);
    ROS_INFO("lidar default max angle: %f", lidar_param_.angle_max / DEG2RAD);
    ROS_INFO("lidar default angle increment: %f", lidar_param_.angle_increment / DEG2RAD);
    ROS_INFO("lidar default min range: %f", lidar_param_.range_min);
    ROS_INFO("lidar default max range: %f", lidar_param_.range_max);
    ROS_INFO("lidar default scan time: %f", lidar_param_.scan_time);
    ROS_INFO("lidar default time increment: %f", lidar_param_.time_increment);
    nh_.param<std::string>("lidar_ip", lidar_ip_, DEFAULT_LIDAR_IP);
    nh_.param<int>("lidar_port", lidar_port_, DEFAULT_LIDAR_PORT);
    nh_.param<int>("host_port", host_port_, DEFAULT_HOST_PORT);
    IP_config_service_ = nh_.advertiseService<osight_lidar::IPConfig::Request, osight_lidar::IPConfig::Response>("ip_cfg", boost::bind(&IExxx::IPCfg, this, _1, _2));
    speed_config_service_ = nh_.advertiseService<osight_lidar::Speed::Request, osight_lidar::Speed::Response>("speed_cfg", boost::bind(&IExxx::speedCfg, this, _1, _2));
    echo_config_service_ = nh_.advertiseService<osight_lidar::Echo::Request, osight_lidar::Echo::Response>("echo_cfg", boost::bind(&IExxx::echoCfg, this, _1, _2));
    outlier_config_service_ = nh_.advertiseService<osight_lidar::Outlier::Request, osight_lidar::Outlier::Response>("outlier_cfg", boost::bind(&IExxx::outlierCfg, this, _1, _2));
    resolution_config_service_ = nh_.advertiseService<osight_lidar::Resolution::Request, osight_lidar::Resolution::Response>("resolution_cfg", boost::bind(&IExxx::resolutionCfg, this, _1, _2));
    intensity_config_service_ = nh_.advertiseService<osight_lidar::Intensity::Request, osight_lidar::Intensity::Response>("intensity_cfg", boost::bind(&IExxx::intensityCfg, this, _1, _2));
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
    req.crc = crc16((uint8_t *)&req, sizeof(req) - 2);
    udp_->send((uint8_t *)&req, sizeof(req));
}

//both host and dev port modification not supported
bool IExxx::IPCfg(osight_lidar::IPConfig::Request &request, osight_lidar::IPConfig::Response &res)
{
    struct IPConfigReq req;
    std::string host_ip, gateway;
    int n;

    ROS_INFO("IP config: dev ip -> %s", request.dev_ip.c_str());

    req.msg_id = htonl(SET_STATIC_IP_REQ);
    req.dev_ip = inet_addr(request.dev_ip.c_str());
    //req.dev_port = inet_addr(request.dev_port.c_str());
    req.dev_port = htonl(DEFAULT_LIDAR_PORT);
    host_ip = request.dev_ip;
    n = host_ip.find_last_of('.') + 1;
    host_ip.replace(n, host_ip.length() - n, "254");

    req.host_ip = inet_addr(host_ip.c_str());
    req.host_port = htonl(DEFAULT_HOST_PORT);
    req.mask = inet_addr("255.255.255.0");

    gateway = request.dev_ip;
    n = gateway.find_last_of('.') + 1;
    gateway.replace(n, gateway.length() - n, "1");
    req.gateway = inet_addr(gateway.c_str());

    cfg_err_ = 0xFFFF;

    req.crc = crc16((uint8_t *)&req, sizeof(req) - 2);
    // for (int i = 0; i < sizeof(req); i++)
    // {
    //     printf("%02x ", ((uint8_t *)&req)[i]);
    // }
    // printf("\r\n");
    udp_->send((uint8_t *)&req, sizeof(req));
    ros::Duration(0.5).sleep();
    res.error_no = cfg_err_;
    ROS_INFO("IP config: %s", err_str(res.error_no));
    ROS_INFO("the new configuration takes effect after \033[33;1mrestarting the lidar and the node\033[0m, please modify the host \033[33;1mIP segment\033[0m according to the IP configuration");
    return true;
}

bool IExxx::speedCfg(osight_lidar::Speed::Request &request, osight_lidar::Speed::Response &res)
{
    struct ParamConfigReq req;

    ROS_INFO("speed config(5-30): %d", request.speed);

    req.msg_id = htonl(PARA_DEVICE_CONFIGURATION_REQ);
    req.speed = request.speed;
    req.intensity = private_param_.intensity;
    req.resolution = htonl(private_param_.resolution);

    cfg_err_ = 0xFFFF;

    req.crc = crc16((uint8_t *)&req, sizeof(req) - 2);
    // for (int i = 0; i < sizeof(req); i++)
    // {
    //     printf("%02x ", ((uint8_t *)&req)[i]);
    // }
    // printf("\r\n");
    udp_->send((uint8_t *)&req, sizeof(req));
    ros::Duration(0.5).sleep();
    res.error_no = cfg_err_;
    ROS_INFO("speed config: %s", err_str(res.error_no));
    if (cfg_err_ == 0)
    {
        private_param_.speed = req.speed;
    }
    return true;
}

bool IExxx::resolutionCfg(osight_lidar::Resolution::Request &request, osight_lidar::Resolution::Response &res)
{
    struct ParamConfigReq req;

    ROS_INFO("resolution config(1250000, 2500000, 3125000, 6250000): %d", request.resolution);

    req.msg_id = htonl(PARA_DEVICE_CONFIGURATION_REQ);
    req.speed = private_param_.speed;
    req.intensity = private_param_.intensity;
    req.resolution = htonl(request.resolution);

    cfg_err_ = 0xFFFF;

    req.crc = crc16((uint8_t *)&req, sizeof(req) - 2);
    // for (int i = 0; i < sizeof(req); i++)
    // {
    //     printf("%02x ", ((uint8_t *)&req)[i]);
    // }
    // printf("\r\n");
    udp_->send((uint8_t *)&req, sizeof(req));
    ros::Duration(0.5).sleep();
    res.error_no = cfg_err_;
    ROS_INFO("resolution config: %s", err_str(res.error_no));
    if (cfg_err_ == 0)
    {
        private_param_.resolution = req.resolution;
    }
    return true;
}

bool IExxx::intensityCfg(osight_lidar::Intensity::Request &request, osight_lidar::Intensity::Response &res)
{
    struct ParamConfigReq req;

    ROS_INFO("intensity config(0,1,2): %d", request.intensity);

    req.msg_id = htonl(PARA_DEVICE_CONFIGURATION_REQ);
    req.speed = private_param_.speed;
    req.intensity = request.intensity;
    req.resolution = htonl(private_param_.resolution);

    cfg_err_ = 0xFFFF;

    req.crc = crc16((uint8_t *)&req, sizeof(req) - 2);
    // for (int i = 0; i < sizeof(req); i++)
    // {
    //     printf("%02x ", ((uint8_t *)&req)[i]);
    // }
    // printf("\r\n");
    udp_->send((uint8_t *)&req, sizeof(req));
    ros::Duration(0.5).sleep();
    res.error_no = cfg_err_;
    ROS_INFO("intensity config: %s", err_str(res.error_no));
    if (cfg_err_ == 0)
    {
        private_param_.intensity = req.intensity;
    }
    return true;
}

bool IExxx::echoCfg(osight_lidar::Echo::Request &request, osight_lidar::Echo::Response &res)
{
    struct FilterConfigReq req;

    ROS_INFO("echo filter config(0,1): %d", request.echo);

    req.msg_id = htonl(ACTIVE_FILTER_REQ);
    req.outlier = private_param_.outlier;
    req.echo = request.echo;
    req.outlier_level = private_param_.outlier_level;

    cfg_err_ = 0xFFFF;

    req.crc = crc16((uint8_t *)&req, sizeof(req) - 2);
    // for (int i = 0; i < sizeof(req); i++)
    // {
    //     printf("%02x ", ((uint8_t *)&req)[i]);
    // }
    // printf("\r\n");
    udp_->send((uint8_t *)&req, sizeof(req));
    ros::Duration(0.5).sleep();
    res.error_no = cfg_err_;
    ROS_INFO("echo filter config: %s", err_str(res.error_no));
    if (cfg_err_ == 0)
    {
        private_param_.echo = req.echo;
    }
    return true;
}

bool IExxx::outlierCfg(osight_lidar::Outlier::Request &request, osight_lidar::Outlier::Response &res)
{
    struct FilterConfigReq req;

    ROS_INFO("outlier filter config(0,1): %d", request.outlier);
    ROS_INFO("outlier level config(1-5): %d", request.outlier_level);
    req.msg_id = htonl(ACTIVE_FILTER_REQ);
    req.outlier = request.outlier;
    req.echo = private_param_.echo;
    req.outlier_level = request.outlier_level;

    cfg_err_ = 0xFFFF;

    req.crc = crc16((uint8_t *)&req, sizeof(req) - 2);
    // for (int i = 0; i < sizeof(req); i++)
    // {
    //     printf("%02x ", ((uint8_t *)&req)[i]);
    // }
    // printf("\r\n");
    udp_->send((uint8_t *)&req, sizeof(req));
    ros::Duration(0.5).sleep();
    res.error_no = cfg_err_;
    ROS_INFO("outlier filter config: %s", err_str(res.error_no));
    if (cfg_err_ == 0)
    {
        private_param_.outlier = req.outlier;
        private_param_.outlier_level = req.outlier_level;
    }
    return true;
}

void IExxx::dataCallback(uint8_t *buff, int len)
{
    uint32_t msg_id = (buff[0] << 24) + (buff[1] << 16) + (buff[2] << 8) + buff[3];
    uint16_t crc = crc16(buff, len - sizeof(uint16_t));
    struct ParamSyncRsp *p_param;
    struct MeasureDataRsp *p_meas;
    uint8_t *p_data;
    int i;
    uint32_t range_data;
    uint32_t intensity_data;
    struct IPConfigRsp *p_ip;
    struct LidarReport *p_report;
    struct ParamConfigRsp *p_param_cfg;
    struct FilterConfigRsp *p_filter_cfg;
    if (crc != (buff[len - 1] << 8) + (buff[len - 2]))
    {
        ROS_WARN("crc error %04x, %02x, %02x\r\n", crc, buff[len - 2], buff[len - 1]);
        return;
    }
    ROS_INFO_ONCE("communication link established.");
    switch (msg_id)
    {
    case PARA_SYNC_RSP:
        ROS_INFO("lidar parameter update.");
        p_param = (struct ParamSyncRsp *)buff;
        if (lidar_param_.angle_min != (float)((int32_t)ntohl(p_param->start_angle) / 1000.0 * DEG2RAD - M_PI / 2))
        {
            lidar_param_.angle_min = (int32_t)ntohl(p_param->start_angle) / 1000.0 * DEG2RAD - M_PI / 2;
            ROS_INFO("lidar min angle update: %f", lidar_param_.angle_min / DEG2RAD);
        }
        if (lidar_param_.angle_max != (float)(ntohl(p_param->stop_angle) / 1000.0 * DEG2RAD - M_PI / 2))
        {
            lidar_param_.angle_max = ntohl(p_param->stop_angle) / 1000.0 * DEG2RAD - M_PI / 2;
            ROS_INFO("lidar max angle update: %f", lidar_param_.angle_max / DEG2RAD);
        }
        if (lidar_param_.angle_increment != (float)(ntohl(p_param->angular_resolution) / 10000000.0 * DEG2RAD))
        {
            lidar_param_.angle_increment = ntohl(p_param->angular_resolution) / 10000000.0 * DEG2RAD;
            ROS_INFO("lidar angle increment update: %f", lidar_param_.angle_increment / DEG2RAD);
        }
        if (lidar_param_.range_max != (float)(ntohl(p_param->max_distance) / 10000.0))
        {
            lidar_param_.range_max = ntohl(p_param->max_distance) / 10000.0;
            ROS_INFO("lidar max range update: %f", lidar_param_.range_max);
        }
        if (lidar_param_.scan_time != (float)(1.0 / p_param->speed))
        {
            lidar_param_.scan_time = 1.0 / p_param->speed;
            ROS_INFO("lidar scan time update: %f", lidar_param_.scan_time);
        }
        if (lidar_param_.time_increment != (float)(lidar_param_.scan_time / (2 * M_PI / lidar_param_.angle_increment)))
        {
            lidar_param_.time_increment = lidar_param_.scan_time / (2 * M_PI / lidar_param_.angle_increment);
            ROS_INFO("lidar time increment update: %f", lidar_param_.time_increment);
        }
        private_param_.outlier = p_param->outlier_filter;
        private_param_.echo = p_param->echo_filter;
        private_param_.outlier_level = p_param->outlier_level;
        private_param_.speed = p_param->speed;
        private_param_.intensity = p_param->intensity_status;
        private_param_.resolution = ntohl(p_param->angular_resolution);
        break;

    case PARA_CHANGED_IND_RSP:
        updateParam();
        break;

    case PARA_DEVICE_CONFIGURATION_RSP:
        p_param_cfg = (struct ParamConfigRsp *)buff;
        p_param_cfg->msg_id = ntohl(p_param_cfg->msg_id);
        p_param_cfg->error_no = ntohs(p_param_cfg->error_no);
        cfg_err_ = p_param_cfg->error_no;
        break;

    case PARA_ALARM_CONFIGURATION_RSQ:
        break;

    case MEAS_DATA_PACKAGE_RSP:
        p_meas = (struct MeasureDataRsp *)buff;
        p_data = p_meas->data;
        p_meas->msg_id = ntohl(p_meas->msg_id);
        p_meas->serial_num = ntohl(p_meas->serial_num);
        p_meas->product_num = ntohl(p_meas->product_num);
        p_meas->time_stamp = ntohl(p_meas->time_stamp);
        p_meas->start_angle = (int32_t)ntohl(p_meas->start_angle);
        p_meas->stop_angle = (int32_t)ntohl(p_meas->stop_angle);
        p_meas->micro_second = ntohl(p_meas->micro_second);
        p_meas->angular_resolution = ntohl(p_meas->angular_resolution);
        p_meas->vertical_angle = ntohs(p_meas->vertical_angle);
        p_meas->max_point_num = ntohs(p_meas->max_point_num);
        p_meas->current_point_num = ntohs(p_meas->current_point_num);

        if (lidar_param_.angle_min != (float)(p_meas->start_angle / 1000.0 * DEG2RAD - M_PI / 2))
        {
            lidar_param_.angle_min = p_meas->start_angle / 1000.0 * DEG2RAD - M_PI / 2;
            ROS_INFO("lidar min angle update: %f", lidar_param_.angle_min);
        }
        if (lidar_param_.angle_max != (float)(p_meas->stop_angle / 1000.0 * DEG2RAD - M_PI / 2))
        {
            lidar_param_.angle_max = p_meas->stop_angle / 1000.0 * DEG2RAD - M_PI / 2;
            ROS_INFO("lidar max angle update: %f", lidar_param_.angle_max);
        }
        if (lidar_param_.angle_increment != (float)(p_meas->angular_resolution / 10000000.0 * DEG2RAD))
        {
            lidar_param_.angle_increment = p_meas->angular_resolution / 10000000.0 * DEG2RAD;
            ROS_INFO("lidar angle increment update: %f", lidar_param_.angle_increment);
        }

        if (p_meas->pkg_num == 0)
        {
            ranges.clear();
            intensities.clear();
        }

        for (i = 0; i < p_meas->current_point_num; i++)
        {
            UNPACK_4_BYTE(p_data, range_data);
            ranges.push_back(range_data / 10000.0);
            if (p_meas->intensity_status == 2)
            {
                UNPACK_2_BYTE(p_data, intensity_data);
                intensities.push_back(intensity_data);
            }
            else if (p_meas->intensity_status == 1)
            {
                UNPACK_1_BYTE(p_data, intensity_data);
                intensities.push_back(intensity_data);
            }
            else
            {
                intensities.push_back(0);
            }
        }

        if (p_meas->total_pkgs == p_meas->pkg_num + 1)
        {
            lidarDataCallback(ranges, intensities, lidar_param_);
        }
        break;

    case LOG_GET_RSP:
        break;

    case TIME_REPORT_INF:
        p_report = (struct LidarReport *)buff;
        if (lidar_param_.scan_time != (float)(1.0 / p_report->speed))
        {
            lidar_param_.scan_time = 1.0 / p_report->speed;
            ROS_INFO("lidar scan time update: %f", lidar_param_.scan_time);
        }
        break;

    case ACTIVE_FILTER_RSP:
        p_filter_cfg = (struct FilterConfigRsp *)buff;
        p_filter_cfg->msg_id = ntohl(p_filter_cfg->msg_id);
        p_filter_cfg->error_no = ntohs(p_filter_cfg->error_no);
        cfg_err_ = p_filter_cfg->error_no;
        break;

    case SET_CALIBRATION_MODE_RSP:
        break;

    case SET_NET_MODE_RSP:
        break;

    case SET_STATIC_IP_RSP:
        p_ip = (struct IPConfigRsp *)buff;
        p_ip->msg_id = ntohl(p_ip->msg_id);
        p_ip->error_no = ntohs(p_ip->error_no);
        cfg_err_ = p_ip->error_no;
        break;

    default:
        break;
    }
}

const char *IExxx::err_str(uint16_t err_no)
{
    int i;
    for (i = 0; i < sizeof(err) / sizeof(err[0]); i++)
    {
        if (err[i].err_no == err_no)
        {
            return err[i].err_str;
        }
    }
    if (i == sizeof(err) / sizeof(err[0]))
    {
        return "no error code found";
    }
}

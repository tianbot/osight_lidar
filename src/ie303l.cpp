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

#include "ie303l.h"

static const struct IE303lErrStr err[] = {
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

IE303l::IE303l(ros::NodeHandle *nh) : OsightLidar(nh)
{
    ranges.clear();
    intensities.clear();
    nh_.param<float>("range_max", lidar_param_.range_max, IE303L_DEFAULT_MAX_RANGES);
    nh_.param<float>("range_min", lidar_param_.range_min, IE303L_DEFAULT_MIN_RANGES);
    nh_.param<float>("scan_time", lidar_param_.scan_time, IE303L_DEFAULT_SCAN_TIME);
    nh_.param<float>("lidar_angle_min", lidar_param_.angle_min, IE303L_DEFAULT_LIDAR_ANGLE_MIN);
    nh_.param<float>("lidar_angle_max", lidar_param_.angle_max, IE303L_DEFAULT_LIDAR_ANGLE_MAX);
    nh_.param<float>("time_increment", lidar_param_.time_increment, IE303L_DEFAULT_TIME_INCREMENT);
    nh_.param<float>("angle_increment", lidar_param_.angle_increment, IE303L_DEFAULT_ANGLE_INCREMENT);
    ROS_INFO("lidar default min angle: %f", lidar_param_.angle_min);
    ROS_INFO("lidar default max angle: %f", lidar_param_.angle_max);
    ROS_INFO("lidar default angle increment: %f", lidar_param_.angle_increment);
    ROS_INFO("lidar default min range: %f", lidar_param_.range_min);
    ROS_INFO("lidar default max range: %f", lidar_param_.range_max);
    ROS_INFO("lidar default scan time: %f", lidar_param_.scan_time);
    ROS_INFO("lidar default time increment: %f", lidar_param_.time_increment);
    nh_.param<std::string>("lidar_serial", lidar_serial_, IE303L_DEFAULT_LIDAR_SERIAL);

    speed_config_service_ = nh_.advertiseService<osight_lidar::Speed::Request, osight_lidar::Speed::Response>("speed_cfg", boost::bind(&IE303l::speedCfg, this, _1, _2));
    echo_config_service_ = nh_.advertiseService<osight_lidar::Echo::Request, osight_lidar::Echo::Response>("echo_cfg", boost::bind(&IE303l::echoCfg, this, _1, _2));
    outlier_config_service_ = nh_.advertiseService<osight_lidar::Outlier::Request, osight_lidar::Outlier::Response>("outlier_cfg", boost::bind(&IE303l::outlierCfg, this, _1, _2));
    resolution_config_service_ = nh_.advertiseService<osight_lidar::Resolution::Request, osight_lidar::Resolution::Response>("resolution_cfg", boost::bind(&IE303l::resolutionCfg, this, _1, _2));
    intensity_config_service_ = nh_.advertiseService<osight_lidar::Intensity::Request, osight_lidar::Intensity::Response>("intensity_cfg", boost::bind(&IE303l::intensityCfg, this, _1, _2));
}

IE303l::~IE303l()
{
    serial_->close();
    delete serial_;
}

void IE303l::serialDataProc(uint8_t *buff, int len)
{
    static uint8_t state = 0;
    uint8_t *p = buff;
    static vector<uint8_t> recv_msg;
    static uint32_t msg_len;
    while (len != 0)
    {
        switch (state)
        {
        case 0:
            if (*p == PROTOCOL_HEAD_0)
            {
                recv_msg.clear();
                recv_msg.push_back(PROTOCOL_HEAD_0);
                state = 1;
            }
            p++;
            len--;
            break;

        case 1:
            if (*p == PROTOCOL_HEAD_1)
            {
                recv_msg.push_back(PROTOCOL_HEAD_1);
                p++;
                len--;
                state = 2;
            }
            else
            {
                state = 0;
            }
            break;

        case 2: // len
            recv_msg.push_back(*p);
            msg_len = (*p) * 256;
            p++;
            len--;
            state = 3;
            break;

        case 3: // len
            recv_msg.push_back(*p);
            msg_len += *p;
            if (msg_len > 1024 * 10)
            {
                state = 0;
                break;
            }
            msg_len -= 4; //minus head and len field
            p++;
            len--;
            state = 4;
            break;

        case 4: // msg
            recv_msg.push_back(*p);
            p++;
            len--;
            msg_len--;
            if (msg_len == 0)
            {
                if ((recv_msg[recv_msg.size() - 2] == PROTOCOL_TAIL_0) && (recv_msg[recv_msg.size() - 1] == PROTOCOL_TAIL_1))
                {
                    dataCallback(&recv_msg[4], recv_msg.size() - 6);
                }
                state = 0;
            }
            break;

        default:
            state = 0;
            break;
        }
    }
}

bool IE303l::init(void)
{
    serial_ = new Serial();
    if (serial_->open(lidar_serial_.c_str(), 1152000, 0, 8, 1, 'N',
                      boost::bind(&IE303l::serialDataProc, this, _1, _2)) != true)
    {
        ROS_INFO("lidar serial port [%s] open failed", lidar_serial_.c_str());
        return false;
    }
    else
    {
        ROS_INFO("lidar serial port [%s] open successed", lidar_serial_.c_str());
        return true;
    }
}

void IE303l::updateParam(void)
{
    struct IE303lParamSyncReq req;
    req.msg_id = htonl(IE303L_PARA_SYNC_REQ);
    req.crc = crc16((uint8_t *)&req, sizeof(req) - 2);

    uint8_t buff[1024];
    uint32_t offset = 0;
    buff[offset++] = PROTOCOL_HEAD_0;
    buff[offset++] = PROTOCOL_HEAD_1;
    buff[offset++] = ((sizeof(IE303lParamSyncReq) + 6) >> 8) & 0xFF; //add head / len /tail
    buff[offset++] = (sizeof(IE303lParamSyncReq) + 6) & 0xFF;
    memcpy(buff + offset, &req, sizeof(req));
    offset += sizeof(req);
    buff[offset++] = PROTOCOL_TAIL_0;
    buff[offset++] = PROTOCOL_TAIL_1;

    serial_->send(buff, offset);
}

void IE303l::startTransferData(void)
{
    struct IE303lStartMeasureTransmissionReq req;
    req.msg_id = htonl(IE303L_START_MEASURE_TRANSMISSION_REQ);
    req.function_id = IE303L_START_DATA_TRANSFER;
    req.crc = crc16((uint8_t *)&req, sizeof(req) - 2);

    uint8_t buff[1024];
    uint32_t offset = 0;
    buff[offset++] = PROTOCOL_HEAD_0;
    buff[offset++] = PROTOCOL_HEAD_1;
    buff[offset++] = ((sizeof(IE303lStartMeasureTransmissionReq) + 6) >> 8) & 0xFF;
    buff[offset++] = (sizeof(IE303lStartMeasureTransmissionReq) + 6) & 0xFF;
    memcpy(buff + offset, &req, sizeof(req));
    offset += sizeof(req);
    buff[offset++] = PROTOCOL_TAIL_0;
    buff[offset++] = PROTOCOL_TAIL_1;

    serial_->send(buff, offset);
}

void IE303l::stopTransferData(void)
{
    struct IE303lStartMeasureTransmissionReq req;
    req.msg_id = htonl(IE303L_START_MEASURE_TRANSMISSION_REQ);
    req.function_id = IE303L_STOP_DATA_TRANSFER;
    req.crc = crc16((uint8_t *)&req, sizeof(req) - 2);

    uint8_t buff[1024];
    uint32_t offset = 0;
    buff[offset++] = PROTOCOL_HEAD_0;
    buff[offset++] = PROTOCOL_HEAD_1;
    buff[offset++] = ((sizeof(IE303lStartMeasureTransmissionReq) + 6) >> 8) & 0xFF;
    buff[offset++] = (sizeof(IE303lStartMeasureTransmissionReq) + 6) & 0xFF;
    memcpy(buff + offset, &req, sizeof(req));
    offset += sizeof(req);
    buff[offset++] = PROTOCOL_TAIL_0;
    buff[offset++] = PROTOCOL_TAIL_1;

    serial_->send(buff, offset);
}

bool IE303l::speedCfg(osight_lidar::Speed::Request &request, osight_lidar::Speed::Response &res)
{
    struct IE303lParamConfigReq req;

    ROS_INFO("speed config(5-30): %d", request.speed);

    req.msg_id = htonl(IE303L_PARA_DEVICE_CONFIGURATION_REQ);
    req.speed = request.speed;
    req.intensity = private_param_.intensity;
    req.resolution = htonl(private_param_.resolution);

    cfg_err_ = 0xFFFF;

    req.crc = crc16((uint8_t *)&req, sizeof(req) - 2);

    uint8_t buff[1024];
    uint32_t offset = 0;
    buff[offset++] = PROTOCOL_HEAD_0;
    buff[offset++] = PROTOCOL_HEAD_1;
    buff[offset++] = ((sizeof(IE303lParamConfigReq) >> 8) + 6) & 0xFF;
    buff[offset++] = (sizeof(IE303lParamConfigReq) + 6) & 0xFF;
    memcpy(buff + offset, &req, sizeof(req));
    offset += sizeof(req);
    buff[offset++] = PROTOCOL_TAIL_0;
    buff[offset++] = PROTOCOL_TAIL_1;

    serial_->send(buff, offset);

    ros::Duration(0.5).sleep();
    res.error_no = cfg_err_;
    ROS_INFO("speed config: %s", err_str(res.error_no));
    if (cfg_err_ == 0)
    {
        private_param_.speed = req.speed;
    }
    return true;
}

bool IE303l::resolutionCfg(osight_lidar::Resolution::Request &request, osight_lidar::Resolution::Response &res)
{
    struct IE303lParamConfigReq req;

    ROS_INFO("resolution config(1250000, 2500000, 3125000, 6250000): %d", request.resolution);

    req.msg_id = htonl(IE303L_PARA_DEVICE_CONFIGURATION_REQ);
    req.speed = private_param_.speed;
    req.intensity = private_param_.intensity;
    req.resolution = htonl(request.resolution);

    cfg_err_ = 0xFFFF;

    req.crc = crc16((uint8_t *)&req, sizeof(req) - 2);

    uint8_t buff[1024];
    uint32_t offset = 0;
    buff[offset++] = PROTOCOL_HEAD_0;
    buff[offset++] = PROTOCOL_HEAD_1;
    buff[offset++] = ((sizeof(IE303lParamConfigReq) >> 8) + 6) & 0xFF;
    buff[offset++] = (sizeof(IE303lParamConfigReq) + 6) & 0xFF;
    memcpy(buff + offset, &req, sizeof(req));
    offset += sizeof(req);
    buff[offset++] = PROTOCOL_TAIL_0;
    buff[offset++] = PROTOCOL_TAIL_1;

    serial_->send(buff, offset);

    ros::Duration(0.5).sleep();
    res.error_no = cfg_err_;
    ROS_INFO("resolution config: %s", err_str(res.error_no));
    if (cfg_err_ == 0)
    {
        private_param_.resolution = req.resolution;
    }
    return true;
}

bool IE303l::intensityCfg(osight_lidar::Intensity::Request &request, osight_lidar::Intensity::Response &res)
{
    struct IE303lParamConfigReq req;

    ROS_INFO("intensity config(0,1,2): %d", request.intensity);

    req.msg_id = htonl(IE303L_PARA_DEVICE_CONFIGURATION_REQ);
    req.speed = private_param_.speed;
    req.intensity = request.intensity;
    req.resolution = htonl(private_param_.resolution);

    cfg_err_ = 0xFFFF;

    req.crc = crc16((uint8_t *)&req, sizeof(req) - 2);

    uint8_t buff[1024];
    uint32_t offset = 0;
    buff[offset++] = PROTOCOL_HEAD_0;
    buff[offset++] = PROTOCOL_HEAD_1;
    buff[offset++] = ((sizeof(IE303lParamConfigReq) >> 8) + 6) & 0xFF;
    buff[offset++] = (sizeof(IE303lParamConfigReq) + 6) & 0xFF;
    memcpy(buff + offset, &req, sizeof(req));
    offset += sizeof(req);
    buff[offset++] = PROTOCOL_TAIL_0;
    buff[offset++] = PROTOCOL_TAIL_1;

    serial_->send(buff, offset);

    ros::Duration(0.5).sleep();
    res.error_no = cfg_err_;
    ROS_INFO("intensity config: %s", err_str(res.error_no));
    if (cfg_err_ == 0)
    {
        private_param_.intensity = req.intensity;
    }
    return true;
}

bool IE303l::echoCfg(osight_lidar::Echo::Request &request, osight_lidar::Echo::Response &res)
{
    struct IE303lFilterConfigReq req;

    ROS_INFO("echo filter config(0,1): %d", request.echo);

    req.msg_id = htonl(IE303L_ACTIVE_FILTER_REQ);
    req.outlier = private_param_.outlier;
    req.echo = request.echo;
    req.outlier_level = private_param_.outlier_level;

    cfg_err_ = 0xFFFF;

    req.crc = crc16((uint8_t *)&req, sizeof(req) - 2);

    uint8_t buff[1024];
    uint32_t offset = 0;
    buff[offset++] = PROTOCOL_HEAD_0;
    buff[offset++] = PROTOCOL_HEAD_1;
    buff[offset++] = ((sizeof(IE303lFilterConfigReq) >> 8) + 6) & 0xFF;
    buff[offset++] = (sizeof(IE303lFilterConfigReq) + 6) & 0xFF;
    memcpy(buff + offset, &req, sizeof(req));
    offset += sizeof(req);
    buff[offset++] = PROTOCOL_TAIL_0;
    buff[offset++] = PROTOCOL_TAIL_1;

    serial_->send(buff, offset);

    ros::Duration(0.5).sleep();
    res.error_no = cfg_err_;
    ROS_INFO("echo filter config: %s", err_str(res.error_no));
    if (cfg_err_ == 0)
    {
        private_param_.echo = req.echo;
    }
    return true;
}

bool IE303l::outlierCfg(osight_lidar::Outlier::Request &request, osight_lidar::Outlier::Response &res)
{
    struct IE303lFilterConfigReq req;

    ROS_INFO("outlier filter config(0,1): %d", request.outlier);
    ROS_INFO("outlier level config(1-5): %d", request.outlier_level);
    req.msg_id = htonl(IE303L_ACTIVE_FILTER_REQ);
    req.outlier = request.outlier;
    req.echo = private_param_.echo;
    req.outlier_level = request.outlier_level;

    cfg_err_ = 0xFFFF;

    req.crc = crc16((uint8_t *)&req, sizeof(req) - 2);

    uint8_t buff[1024];
    uint32_t offset = 0;
    buff[offset++] = PROTOCOL_HEAD_0;
    buff[offset++] = PROTOCOL_HEAD_1;
    buff[offset++] = ((sizeof(IE303lFilterConfigReq) >> 8) + 6) & 0xFF;
    buff[offset++] = (sizeof(IE303lFilterConfigReq) + 6) & 0xFF;
    memcpy(buff + offset, &req, sizeof(req));
    offset += sizeof(req);
    buff[offset++] = PROTOCOL_TAIL_0;
    buff[offset++] = PROTOCOL_TAIL_1;

    serial_->send(buff, offset);

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

void IE303l::dataCallback(uint8_t *buff, int len)
{
    uint32_t msg_id = (buff[0] << 24) + (buff[1] << 16) + (buff[2] << 8) + buff[3];
    uint16_t crc = crc16(buff, len - sizeof(uint16_t));
    struct IE303lParamSyncRsp *p_param;
    struct IE303lMeasureDataRsp *p_meas;
    uint8_t *p_data;
    int i;
    uint32_t range_data;
    uint32_t intensity_data;

    struct IE303lLidarReport *p_report;
    struct IE303lParamConfigRsp *p_param_cfg;
    struct IE303lFilterConfigRsp *p_filter_cfg;
    if (crc != (buff[len - 1] << 8) + (buff[len - 2]))
    {
        ROS_WARN("crc error %04x, %02x, %02x\r\n", crc, buff[len - 2], buff[len - 1]);
        return;
    }
    ROS_INFO_ONCE("communication link established.");
    switch (msg_id)
    {
    case IE303L_PARA_SYNC_RSP:
        ROS_INFO("lidar parameter update.");
        p_param = (struct IE303lParamSyncRsp *)buff;
        if (lidar_param_.angle_min != (float)((int32_t)ntohl(p_param->start_angle) / 1000.0 * DEG2RAD - M_PI / 2))
        {
            lidar_param_.angle_min = (int32_t)ntohl(p_param->start_angle) / 1000.0 * DEG2RAD - M_PI / 2;
            ROS_INFO("lidar min angle update: %f", lidar_param_.angle_min);
        }
        if (angle_min_ < lidar_param_.angle_min)
        {
            angle_min_ = lidar_param_.angle_min;
        }

        if (lidar_param_.angle_max != (float)(ntohl(p_param->stop_angle) / 1000.0 * DEG2RAD - M_PI / 2))
        {
            lidar_param_.angle_max = ntohl(p_param->stop_angle) / 1000.0 * DEG2RAD - M_PI / 2;
            ROS_INFO("lidar max angle update: %f", lidar_param_.angle_max);
        }
        if (angle_max_ > lidar_param_.angle_max)
        {
            angle_max_ = lidar_param_.angle_max;
        }
        if (lidar_param_.angle_increment != (float)(ntohl(p_param->angular_resolution) / 10000000.0 * DEG2RAD))
        {
            lidar_param_.angle_increment = ntohl(p_param->angular_resolution) / 10000000.0 * DEG2RAD;
            ROS_INFO("lidar angle increment update: %f", lidar_param_.angle_increment);
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

    case IE303L_PARA_DEVICE_CONFIGURATION_RSP:
        p_param_cfg = (struct IE303lParamConfigRsp *)buff;
        p_param_cfg->msg_id = ntohl(p_param_cfg->msg_id);
        p_param_cfg->error_no = ntohs(p_param_cfg->error_no);
        cfg_err_ = p_param_cfg->error_no;
        break;

    case IE303L_MEAS_DATA_PACKAGE_RSP:
        p_meas = (struct IE303lMeasureDataRsp *)buff;
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

    case IE303L_TIME_REPORT_INF:
        p_report = (struct IE303lLidarReport *)buff;
        if (lidar_param_.scan_time != (float)(1.0 / p_report->speed))
        {
            lidar_param_.scan_time = 1.0 / p_report->speed;
            ROS_INFO("lidar scan time update: %f", lidar_param_.scan_time);
        }
        break;

    case IE303L_ACTIVE_FILTER_RSP:
        p_filter_cfg = (struct IE303lFilterConfigRsp *)buff;
        p_filter_cfg->msg_id = ntohl(p_filter_cfg->msg_id);
        p_filter_cfg->error_no = ntohs(p_filter_cfg->error_no);
        cfg_err_ = p_filter_cfg->error_no;
        break;

    default:
        break;
    }
}

const char *IE303l::err_str(uint16_t err_no)
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

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

#ifndef _IE303L_H_
#define _IE303L_H_

#include "osight_lidar.h"
#include "serial.h"
#include "osight_lidar/Echo.h"
#include "osight_lidar/Intensity.h"
#include "osight_lidar/Outlier.h"
#include "osight_lidar/Speed.h"
#include "osight_lidar/Resolution.h"
#include <arpa/inet.h> 

#define PROTOCOL_HEAD_0 'I'
#define PROTOCOL_HEAD_1 'E'
#define PROTOCOL_TAIL_0 'O'
#define PROTOCOL_TAIL_1 'S'

#define IE303L_DEFAULT_LIDAR_SERIAL "/dev/ttyUSB0"


#define IE303L_DEFAULT_MAX_RANGES ((float)210000 / 10000)
#define IE303L_DEFAULT_MIN_RANGES ((float)0.01)
#define IE303L_DEFAULT_LIDAR_ANGLE_MIN ((float)(-45.0 * DEG2RAD - M_PI / 2))
#define IE303L_DEFAULT_LIDAR_ANGLE_MAX ((float)(225.0 * DEG2RAD - M_PI / 2))
#define IE303L_DEFAULT_ANGLE_INCREMENT ((float)(0.25 * DEG2RAD))
#define IE303L_DEFAULT_SCAN_TIME ((float)(1 / 25.0))
#define IE303L_DEFAULT_TIME_INCREMENT ((float)(IE303L_DEFAULT_SCAN_TIME / (360 / 0.25)))

enum IE303L_MSG_ID
{
    IE303L_PARA_SYNC_REQ = 0x02000001,
    IE303L_PARA_SYNC_RSP = 0x02000002,
    IE303L_PARA_DEVICE_CONFIGURATION_REQ = 0x02000201,
    IE303L_PARA_DEVICE_CONFIGURATION_RSP = 0x02000202,
    IE303L_START_MEASURE_TRANSMISSION_REQ = 0x02000401,
    IE303L_MEAS_DATA_PACKAGE_RSP = 0x02000402,
    IE303L_TIME_REPORT_INF = 0x02000602,
    IE303L_ACTIVE_FILTER_REQ = 0x02000701,
    IE303L_ACTIVE_FILTER_RSP = 0x02000702,
    IE303L_ANY_SHAPE_ALARM_PARA_REQ = 0x02001501,
    IE303L_ANY_SHAPE_ALARM_PARA_RSP = 0x02001502,
    IE303L_ALARM_PARA_REQ = 0x02001601,
    IE303L_ALARM_PARA_REQ_RSP = 0x02001602,
    IE303L_ALARM_FILTER_PARA_CONFIG_REQ = 0x02001701,
    IE303L_ALARM_FILTER_PARA_CONFIG_RSP = 0x02001702,
    IE303L_GET_ALARM_FILTER_PARA_REQ = 0x02001801,
    IE303L_GET_ALARM_FILTER_PARA_RSP = 0x02001802,
};

#define IE303L_START_DATA_TRANSFER 1
#define IE303L_STOP_DATA_TRANSFER 2

#pragma pack(push)
#pragma pack(1)

struct IE303lParamSyncReq
{
    uint32_t msg_id;
    uint16_t crc;
};

struct IE303lParamSyncRsp
{
    uint32_t msg_id;
    uint8_t mac[6];
    uint32_t serila_num;
    uint32_t product_num;
    uint16_t software_version;
    uint16_t fpga_version;
    uint8_t tdc_type;
    uint8_t line_num;
    int32_t start_angle;
    int32_t stop_angle;
    uint32_t point_num;
    uint32_t max_distance;
    uint8_t net_mode;
    uint8_t dev_ip[4];
    uint32_t dev_port;
    uint8_t host_ip[4];
    uint32_t host_port;
    uint8_t net_mask[4];
    uint8_t gateway[4];
    uint8_t speed;
    uint8_t intensity_status;
    uint32_t angular_resolution;
    uint16_t vertical_angle;
    uint8_t outlier_filter;
    uint8_t echo_filter;
    uint8_t current_area_id;
    uint8_t alarm_type;
    uint8_t alarm_para;
    uint8_t outlier_level;
    uint16_t crc;
};

struct IE303lParaConfigurationReq
{
    uint32_t msg_id;
    uint8_t speed;
    uint8_t intensity_status;
    uint32_t angular_resolution;
    uint16_t crc;
};

struct IE303lParaConfigurationRsp
{
    uint32_t msg_id;
    uint16_t error_code;
    uint16_t crc;
};

struct IE303lStartMeasureTransmissionReq
{
    uint32_t msg_id;
    uint8_t function_id;
    uint16_t crc;
};

struct IE303lMeasureDataRsp
{
    uint32_t msg_id;
    uint8_t line_num;
    uint8_t echo;
    uint32_t serial_num;
    uint32_t product_num;
    uint8_t intensity_status;
    uint8_t device_status;
    uint32_t time_stamp;
    uint8_t inputs_status[4];
    uint8_t outputs_status[4];
    int32_t start_angle;
    int32_t stop_angle;
    uint8_t year;
    uint8_t month;
    uint8_t date;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint32_t micro_second;
    uint8_t reserve[32];
    uint32_t angular_resolution;
    uint16_t vertical_angle;
    uint8_t total_pkgs;
    uint16_t max_point_num;
    uint8_t pkg_num;
    uint16_t current_point_num;
    uint8_t data[]; //the last 2 byte are crc code
};

struct IE303lParamConfigReq
{
    uint32_t msg_id;
    uint8_t speed;
    uint8_t intensity;
    uint32_t resolution;
    uint16_t crc;
};

struct IE303lParamConfigRsp
{
    uint32_t msg_id;
    uint16_t error_no;
    uint16_t crc;
};

struct IE303lFilterConfigReq
{
    uint32_t msg_id;
    uint8_t outlier;
    uint8_t echo;
    uint8_t outlier_level;
    uint16_t crc;
};

struct IE303lFilterConfigRsp
{
    uint32_t msg_id;
    uint16_t error_no;
    uint16_t crc;
};

struct IE303lLidarReport
{
    uint32_t msg_id;
    uint8_t speed;
    uint32_t mcu_vol;
    int32_t mcu_temp;
    int32_t dev_temp;
    uint32_t dev_humidity;
    uint16_t crc;
};

struct IE303lErrStr
{
    uint16_t err_no;
    char const *err_str;
};

struct IE303lPrivateParam
{
    uint8_t outlier;
    uint8_t echo;
    uint8_t outlier_level;
    uint8_t speed;
    uint8_t intensity;
    uint32_t resolution;
};

#pragma pack(pop)

class IE303l : public OsightLidar
{
public:
    IE303l(ros::NodeHandle *nh);
    ~IE303l();
    virtual bool init(void);
    virtual void updateParam(void);
    virtual void startTransferData(void);
    virtual void stopTransferData(void);

    bool speedCfg(osight_lidar::Speed::Request &req, osight_lidar::Speed::Response &res);
    bool echoCfg(osight_lidar::Echo::Request &req, osight_lidar::Echo::Response &res);
    bool outlierCfg(osight_lidar::Outlier::Request &req, osight_lidar::Outlier::Response &res);
    bool resolutionCfg(osight_lidar::Resolution::Request &req, osight_lidar::Resolution::Response &res);
    bool intensityCfg(osight_lidar::Intensity::Request &req, osight_lidar::Intensity::Response &res);
    const char *err_str(uint16_t err_no);

private:
    void dataCallback(uint8_t *buff, int len);
    void serialDataProc(uint8_t *buff, int len);
    vector<float> ranges;
    vector<float> intensities;
    ros::ServiceServer speed_config_service_;
    ros::ServiceServer echo_config_service_;
    ros::ServiceServer outlier_config_service_;
    ros::ServiceServer resolution_config_service_;
    ros::ServiceServer intensity_config_service_;
    std::string lidar_serial_;
    uint16_t cfg_err_;
    Serial *serial_;
    struct IE303lPrivateParam private_param_;
};

#endif

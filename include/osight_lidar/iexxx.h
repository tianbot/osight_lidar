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

#ifndef _IEXXX_H_
#define _IEXXX_H_

#include "osight_lidar.h"
#include "udp.h"

#define DEFAULT_MAX_RANGES 500
#define DEFAULT_MIN_RANGES 0.01
#define DEFAULT_ANGLE_MIN (-45.0 * DEG2RAD - M_PI / 2)
#define DEFAULT_ANGLE_MAX (225.0 * DEG2RAD - M_PI / 2)
#define DEFAULT_ANGLE_INCREMENT (0.25 * DEG2RAD)
#define DEFAULT_SCAN_TIME (1/25.0)
#define DEFAULT_TIME_INCREMENT (DEFAULT_SCAN_TIME/(360/0.25))

enum IEXXX_MSG_ID
{
    PARA_SYNC_REQ = 0x02000001,
    PARA_SYNC_RSP = 0x02000002,
    PARA_CHANGED_IND_RSP = 0x02000102,
    PARA_DEVICE_CONFIGURATION_REQ = 0x02000201,
    PARA_DEVICE_CONFIGURATION_RSP = 0x02000202,
    PARA_ALARM_CONFIGURATION_REQ = 0x02000301,
    PARA_ALARM_CONFIGURATION_RSQ = 0x02000302,
    START_MEASURE_TRANSMISSION_REQ = 0x02000401,
    MEAS_DATA_PACKAGE_RSP = 0x02000402,
    LOG_GET_REQ = 0x02000501,
    LOG_GET_RSP = 0x02000502,
    TIME_REPORT_INF = 0x02000602,
    ACTIVE_FILTER_REQ = 0x02000701,
    ACTIVE_FILTER_RSP = 0x02000702,
    SET_CALIBRATION_MODE_REQ = 0x02000801,
    SET_CALIBRATION_MODE_RSP = 0x02000802,
    SET_NET_MODE_REQ = 0x02000901,
    SET_NET_MODE_RSP = 0x02000902,
    SET_STATIC_IP_REQ = 0x02000A01,
    SET_STATIC_IP_RSP = 0x02000A02,
};
#pragma pack(push)
#pragma pack(1)

struct ParamSyncReq
{
    uint32_t msg_id;
    uint16_t crc;
};

struct ParamSyncRsp
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

struct ParaConfigurationReq
{
    uint32_t msg_id;
    uint8_t speed;
    uint8_t intensity_status;
    uint32_t angular_resolution;
    uint16_t crc;
};

struct ParaConfigurationRsp
{
    uint32_t msg_id;
    uint16_t error_code;
    uint16_t crc;
};

struct StartMeasureTransmissionReq
{
    uint32_t msg_id;
    uint8_t function_id;
    uint16_t crc;
};

struct MeasureDataRsp
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
#pragma pack(pop)

class IExxx : public OsightLidar
{
public:
    IExxx(ros::NodeHandle *nh);
    ~IExxx();
    virtual bool init(void);
    virtual void updateParam(void);

private:
    void dataCallback(uint8_t *buff, int len);
    Udp udp_;
    vector<float> ranges;
    vector<float> intensities;
};

#endif

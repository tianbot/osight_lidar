#ifndef _OSIGHT_LIDAR_H_
#define _OSIGHT_LIDAR_H_

#include "ros.h"
#include "stdint.h"
#include "sensor_msgs/LaserScan.h"

#define DEFAULT_LIDAR_MODEL "iexxx"

#define DEFAULT_FRAME_ID "laser"

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
    ~OsightLidar();

protected:
    ros::NodeHandle *nh_;
    void lidarDataCallback(vector<float> ranges, vector<float> intensities, struct LidarParam lidar_param);

private:
    uint32_t seq;
    std::string frame_id_;
};

#endif

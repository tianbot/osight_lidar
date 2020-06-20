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
    scan_pub_.Publish(scan_msg);
}
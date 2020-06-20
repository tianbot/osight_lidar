#include "iexxx.h"

IExxx::IExxx(ros::NodeHandle *nh) : nh_(*nh)
{
    seq_ = 0;
    nh_.param<std::string>("frame_id", frame_id_, DEFAULT_FRAME_ID);
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1);
}

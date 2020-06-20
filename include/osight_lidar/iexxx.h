#include "osight_lidar.h"

#define DEFAULT_LIDAR_IP "192.168.1.10"
#define DEFAULT_LIDAR_PORT 6500
#define DEFAULT_HOST_PORT 5500

class IExxx : public OsightLidar
{
public:
    IExxx(ros::NodeHandle *nh);
    ~IExxx();
private:
    
    struct LidarParam iexxx_param_;
};
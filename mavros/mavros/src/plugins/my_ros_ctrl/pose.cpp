#include "mavros_msgs/Disk.h"
#include "ros/ros.h"
#include "robot_pose_ekf/GetStatus.h"

class DiskHandle
{
    private:
        ros::NodeHandle nh;
        
    public:
        DiskHandle();
};

int main(int argc, char**argv)
{
    ros::init(argc,argv,"talker");
    DiskHandle dh;
    ros::spin();
    return 0;
}

#include "joy_pub/up_machine.h"


int main(int argc,char*argv[])
{
    ros::init(argc, argv, "speed_plan_node");
    up_machine_ns::Speed_plan _speed_plan;

    ros::AsyncSpinner spinner(0);
    spinner.start();
    // ros::spin();
    ros::waitForShutdown();

    return 0;
}
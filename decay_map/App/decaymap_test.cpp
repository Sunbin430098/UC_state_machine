//目标：开雷达，扫，本来没有，放，有有有
#include "plan_env/decay_map.hpp"

DecayMap::Ptr dcm_ptr;

shared_ptr<DecayMapConfig> dcm_cfg_ptr;

int main(int argc, char **argv) {
    ros::init(argc, argv, "decay_map");

    ros::NodeHandle nh("~");

    dcm_cfg_ptr.reset(new DecayMapConfig(nh));

    dcm_ptr.reset(new DecayMap(nh, *dcm_cfg_ptr));

    Vec3 a(0,0,2);
    bool t = dcm_ptr->isOccupied(a);
    std::cout<<"t = "<<t<<std::endl;

    Eigen::Vector3d minbox_size(0,0,0);
    Eigen::Vector3d maxbox_size(1,1,1);
    Eigen::Vector3d b(0,0,0);
    Eigen::Vector3d c(-1,0,1);
    vector<Eigen::Vector3d>  points;
    points.push_back(b);
    points.push_back(c);
    dcm_ptr->boxSearch(minbox_size,maxbox_size,points);
    for (const auto& point : points) {
    std::cout << "x: " << point.x() << ", y: " << point.y() << ", z: " << point.z() << std::endl;
}

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();
    ros::waitForShutdown();
    return 0;
}


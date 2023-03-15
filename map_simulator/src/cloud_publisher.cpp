#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include "pthread.h"
#include <tf/transform_broadcaster.h>
#include<pcl/io/pcd_io.h>

#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "data/"+name))


using namespace std;
using namespace Eigen;
ros::Publisher local_pc_pub;
sensor_msgs::PointCloud2 localMap_pcd;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
string filename, pub_topic;
double shift_x, shift_y, shift_z;
string frame_id;






void pubSensedPoints(const ros::TimerEvent &e) {
//    static int first = 0;
//    if(first) return;
    localMap_pcd.header.frame_id = frame_id;
    localMap_pcd.header.stamp = ros::Time::now();
    local_pc_pub.publish(localMap_pcd);
//    first ++;
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "cloud_publisher");
    ros::NodeHandle nh("~");
    tf::TransformBroadcaster b;
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    nh.param("frame_id", frame_id, string("world"));
    nh.param("topic", pub_topic, string("/map_generator/global_cloud"));
    nh.param("file", filename, string("zhangjiajie.pcd"));
    nh.param("shift_x", shift_x, 0.0);
    nh.param("shift_y", shift_y, 0.0);
    nh.param("shift_z", shift_z, 0.0);

    pcl::io::loadPCDFile (DEBUG_FILE_DIR(filename), *cloud_);

    Matrix4f Gmat = Matrix4f::Identity();
    Gmat(0,3) = shift_x;
    Gmat(1,3) = shift_y;
    Gmat(2,3) = shift_z;
    pcl::transformPointCloud(*cloud_, *cloud_, Gmat);

    pcl::toROSMsg(*cloud_, localMap_pcd);

    local_pc_pub = nh.advertise<sensor_msgs::PointCloud2>(pub_topic.c_str(), 1, true);
    ros::Timer pub_timer = nh.createTimer(ros::Duration(10.0), pubSensedPoints);
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(10.0).sleep();
    ros::waitForShutdown();
    return 0;
}

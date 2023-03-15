#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include "pthread.h"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/point_cloud_conversion.h"

#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "data/"+name))

pthread_mutex_t mutex;
bool has_odom = false;


using namespace std;;
typedef Eigen::Matrix<double, 3, 3> Mat33;
typedef Eigen::Matrix<double, 3, 1> Vec3;
ros::Publisher local_pc_pub;
ros::Subscriber global_pc_sub;
bool is_map = false;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>

pcl::PointCloud<pcl::PointXYZ> cloud;
void globalMapCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ> pc;

    //    pcl::PCLPointCloud2 pcl_pc2;


    pcl::fromROSMsg(*msg, pc);
    cloud.insert(cloud.end(), pc.begin(), pc.end());
    std::cout << "add pc:" << pc.size() << std::endl;


}


int main(int argc, char **argv) {
    ros::init(argc, argv, "cloud_recorder");
    ros::NodeHandle nh("~");
    tf::TransformBroadcaster b;
    string origin_cloud;
    string pcd_file;
    nh.getParam("cloud_topic", origin_cloud);
    nh.getParam("file_name", pcd_file);
    global_pc_sub = nh.subscribe(origin_cloud, 99, globalMapCallback);
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(10.0).sleep();
    ros::waitForShutdown();
    pcl::io::savePCDFileASCII (DEBUG_FILE_DIR(pcd_file), cloud);//保存p
    return 0;
}

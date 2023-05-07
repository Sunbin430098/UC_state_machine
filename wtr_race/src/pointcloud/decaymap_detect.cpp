// //直接读取地图，不好说经过降采样维护地图之后还是否需要直接读取地图，先留着(也没有地图变换)
// // #include <ros/ros.h>
// // #include <pcl_ros/point_cloud.h>
// // #include <pcl_conversions/pcl_conversions.h>
// // #include <pcl/io/pcd_io.h>
// // #include <pcl/octree/octree.h>
// // #include <Eigen/Geometry>
// // #include <pcl/common/transforms.h>
// // typedef pcl::PointXYZ PointType;
// // int main(int argc, char **argv)
// // {
// //     ros::init(argc,argv,"pointcloud");
// //     ros::NodeHandle nh;
// //     // 读取点云数据
// //     sensor_msgs::PointCloud2ConstPtr input_cloud_msg;
// //     pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
// //     pcl::fromROSMsg(*input_cloud_msg, *input_cloud);
// //     pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
// //     pcl::io::loadPCDFile<PointType>("/home/ubuntu/wtr_upmachine_ws/src/wtr_race/data/t3.pcd", *cloud);
// //     pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);  // 创建一个新点云
// //     Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity(); // 创建变换矩阵
// //     transform_matrix.translation() << 0.0, 0.0, 0.0; // 设置平移矩阵
// //     pcl::transformPointCloud(*cloud, *transformed_cloud, transform_matrix); // 将点云进行变换
// //     // 创建八叉树对象
// //     float resolution = 0.1; // 八叉树分辨率
// //     pcl::octree::OctreePointCloudSearch<PointType> octree(resolution);
// //     octree.setInputCloud(cloud);
// //     octree.addPointsFromInputCloud();
// //     // 判断某一点是否有点云
// //     PointType searchPoint;
// //     searchPoint.x = 0.0;
// //     searchPoint.y = -33.0;
// //     searchPoint.z = 0.0;
// //     float searchRadius = resolution * 0.5f; // 搜索半径为八叉树分辨率的一半
// //     std::vector<int> pointIdxVec;
// //     if (octree.voxelSearch(searchPoint, pointIdxVec))
// //     {
// //         std::cout << "Point has nearby points within search radius" << std::endl;
// //     }
// //     else
// //     {
// //         std::cout << "Point has no nearby points within search radius" << std::endl;
// //     }
// //     return 0;
// // }

// //需要遍历所有data，数据量太大
// // #include <ros/ros.h>
// // #include <sensor_msgs/PointCloud2.h>
// // int main(int argc, char** argv)
// // {
// //     ros::init(argc, argv, "pointcloud_example");
// //     ros::NodeHandle nh;
// //     // 订阅点云话题
// //     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/pointcloud_topic", 1, 
// //         [&](const sensor_msgs::PointCloud2::ConstPtr& msg) {
// //             // 需要检查的点的坐标
// //             float x = 1.0;
// //             float y = 2.0;
// //             float z = 3.0;
// //             // 检索点云中的数据
// //             const uint8_t* ptr = &msg->data[0];
// //             float point_x, point_y, point_z;
// //             uint32_t point_offset = msg->point_step;
// //             for (size_t i = 0; i < msg->width * msg->height; ++i)
// //             {
// //                 memcpy(&point_x, ptr + msg->fields[0].offset, sizeof(float));
// //                 memcpy(&point_y, ptr + msg->fields[1].offset, sizeof(float));
// //                 memcpy(&point_z, ptr + msg->fields[2].offset, sizeof(float));
// //                 // 判断给定坐标是否存在点云
// //                 if (point_x == x && point_y == y && point_z == z)
// //                 {
// //                     ROS_INFO("Point cloud exists at (%f, %f, %f)", x, y, z);
// //                     return;
// //                 }
// //                 ptr += point_offset;
// //             }
// //             ROS_INFO("No point cloud at (%f, %f, %f)", x, y, z);
// //         });
// //     ros::spin();
// //     return 0;
// // }

// //缺少地图到点云关于坐标系的变换，最后目的是与4结合用起来
// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl/point_types.h>
// #include <pcl/octree/octree.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/octree/octree.h>
// #include <Eigen/Geometry>
// #include <pcl/common/transforms.h>
// #include <pcl_ros/transforms.h>
// #include <tf/transform_listener.h>
// // 定义点云类型
// typedef pcl::PointXYZ PointT;
// // 定义Octree数据结构
// pcl::octree::OctreePointCloudSearch<PointT> octree(1.0);
// void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
// {
//     std::cout<<"hi"<<std::endl;
//     // 将消息转换为PCL点云类型
//     pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
//     pcl::fromROSMsg(*msg, *cloud);
//     // 构建Octree
//     octree.setInputCloud(cloud);
//     octree.addPointsFromInputCloud();
//     // 判断某一点是否有点云
//     float resolution = 0.1;
//     PointT searchPoint;
//     searchPoint.x = 0.0;
//     searchPoint.y = -33.0;
//     searchPoint.z = 0.0;
//     float searchRadius = resolution * 0.5f; // 搜索半径为八叉树分辨率的一半
//     std::vector<int> pointIdxVec;
//     if (octree.voxelSearch(searchPoint, pointIdxVec))
//     {
//         std::cout << "Point has nearby points within search radius" << std::endl;
//     }
//     else
//     {
//         std::cout << "Point has no nearby points within search radius" << std::endl;
//     }
// }
// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "pointcloud_node");
//     ros::NodeHandle nh;
//     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/t3_cloud", 1, cloudCallback);
//     ros::spin();
//     return 0;
// }
// void map2pclmsg(ros::NodeHandle nh)
// {
//     // 创建一个tf监听器
//     tf::TransformListener listener;
//     // 创建一个发布器，用于发布转换后的点云数据
//     ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_pointcloud", 1);
//     // 创建一个订阅器，用于订阅雷达发布的点云数据
//     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("lidar_pointcloud", 1, [&](const sensor_msgs::PointCloud2ConstPtr& msg){
//     // 将sensor_msgs/PointCloud2类型的数据转换成pcl::PointCloud<PointT>类型的数据
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromROSMsg(*msg, *cloud);
//     // 将点云从雷达坐标系转换到地图坐标系
//     tf::StampedTransform transform;
//     try{
//         // 获雷达坐标系到地图坐标系的变换
//         listener.lookupTransform("map", "lidar", ros::Time(0), transform);
//     }
//     catch (tf::TransformException ex){
//         ROS_ERROR("%s",ex.what());
//         return;
//     }
//     // 将点云进行坐标系变换
//     pcl_ros::transformPointCloud(*cloud, *cloud, transform);
//     // 将pcl::PointCloud<PointT>类型的数据转换成sensor_msgs/PointCloud2类型的数据
//     sensor_msgs::PointCloud2 output;
//     pcl::toROSMsg(*cloud, output);
//     // 发布转换后的点云数据
//     pub.publish(output);
//     });
// }

#include "livox_lidar/plan_env/decay_map.hpp"

class LivoxDetect
{
    public:
        LivoxDetect(ros::NodeHandle &n);
        void lidarCallback(const ros::TimerEvent&);

    private:
        ros::NodeHandle nh;
        ros::Timer timer;
        DecayMap::Ptr dcm_ptr;
        shared_ptr<DecayMapConfig> dcm_cfg_ptr;
};

LivoxDetect::LivoxDetect(ros::NodeHandle &n)
{
    nh = n;
    timer = nh.createTimer(ros::Duration(0.5), &LivoxDetect::lidarCallback,this);
    dcm_cfg_ptr.reset(new DecayMapConfig(nh));
    dcm_ptr.reset(new DecayMap(nh, *dcm_cfg_ptr));

}

void LivoxDetect::lidarCallback(const ros::TimerEvent&)
{
    // Vec3 a(20,20,1);
    // Vec3 a(-0.4,-0.4,0.15);
    // bool t = dcm_ptr->isOccupied(a);
    // std::cout<<"t = "<<t<<std::endl;
    Eigen::Vector3d maxbox_size(8,8,2);
    Eigen::Vector3d minbox_size(-8,-8,0);
    vector<Eigen::Vector3d>  points;

    dcm_ptr->boxSearchInflate(maxbox_size,minbox_size,points);
    // for (const auto& point : points) 
    // { std::cout << "x: " << point.x() << ", y: " << point.y() << ", z: " << point.z() << std::endl;}
    ROS_INFO("%d",points.size());
    // ROS_INFO("Timer callback triggered");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "decay_map");
    ros::NodeHandle nh("~");
    LivoxDetect livixdetect(nh); 
    // ros::AsyncSpinner spinner(0);
    // spinner.start();
    // ros::waitForShutdown();
    ros::spin();
    return 0;
}


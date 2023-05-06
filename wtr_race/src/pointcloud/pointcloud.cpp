// #include <ros/ros.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/octree/octree.h>
// #include <Eigen/Geometry>
// #include <pcl/common/transforms.h>

// typedef pcl::PointXYZ PointType;

// int main(int argc, char **argv)
// {
//     ros::init(argc,argv,"pointcloud");
//     ros::NodeHandle nh;
//     // 读取点云数据

//     sensor_msgs::PointCloud2ConstPtr input_cloud_msg;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromROSMsg(*input_cloud_msg, *input_cloud);
    
//     pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
//     pcl::io::loadPCDFile<PointType>("/home/ubuntu/wtr_upmachine_ws/src/wtr_race/data/t3.pcd", *cloud);

//     pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);  // 创建一个新点云
//     Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity(); // 创建变换矩阵
//     transform_matrix.translation() << 0.0, 0.0, 0.0; // 设置平移矩阵
//     pcl::transformPointCloud(*cloud, *transformed_cloud, transform_matrix); // 将点云进行变换


//     // 创建八叉树对象
//     float resolution = 0.1; // 八叉树分辨率
//     pcl::octree::OctreePointCloudSearch<PointType> octree(resolution);
//     octree.setInputCloud(cloud);
//     octree.addPointsFromInputCloud();

//     // 判断某一点是否有点云
//     PointType searchPoint;
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

//     return 0;
// }


// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "pointcloud_example");
//     ros::NodeHandle nh;

//     // 订阅点云话题
//     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/pointcloud_topic", 1, 
//         [&](const sensor_msgs::PointCloud2::ConstPtr& msg) {
//             // 需要检查的点的坐标
//             float x = 1.0;
//             float y = 2.0;
//             float z = 3.0;

//             // 检索点云中的数据
//             const uint8_t* ptr = &msg->data[0];
//             float point_x, point_y, point_z;
//             uint32_t point_offset = msg->point_step;
//             for (size_t i = 0; i < msg->width * msg->height; ++i)
//             {
//                 memcpy(&point_x, ptr + msg->fields[0].offset, sizeof(float));
//                 memcpy(&point_y, ptr + msg->fields[1].offset, sizeof(float));
//                 memcpy(&point_z, ptr + msg->fields[2].offset, sizeof(float));
                
//                 // 判断给定坐标是否存在点云
//                 if (point_x == x && point_y == y && point_z == z)
//                 {
//                     ROS_INFO("Point cloud exists at (%f, %f, %f)", x, y, z);
//                     return;
//                 }

//                 ptr += point_offset;
//             }

//             ROS_INFO("No point cloud at (%f, %f, %f)", x, y, z);
//         });

//     ros::spin();
//     return 0;
// }

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
<<<<<<< b7f1d2d145e76c5f1e67a41dc9626a5adc59406f
=======
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

>>>>>>> 视觉模块调整完成，需要最后接受雷达扫到的套上的环的坐标，并通过识别判断环的颜色
// 定义点云类型
typedef pcl::PointXYZ PointT;

// 定义Octree数据结构
pcl::octree::OctreePointCloudSearch<PointT> octree(1.0);

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    std::cout<<"hi"<<std::endl;
    // 将消息转换为PCL点云类型
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);

    // 构建Octree
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    // // 检查某一点是否有点云
    // PointT searchPoint;
    // searchPoint.x = 1.0;  // 检查的点的坐标
    // searchPoint.y = 2.0;
    // searchPoint.z = 3.0;

    // 判断某一点是否有点云
    float resolution = 0.1;
    PointT searchPoint;
    searchPoint.x = 0.0;
    searchPoint.y = -33.0;
    searchPoint.z = 0.0;
    float searchRadius = resolution * 0.5f; // 搜索半径为八叉树分辨率的一半
    std::vector<int> pointIdxVec;
    if (octree.voxelSearch(searchPoint, pointIdxVec))
    {
        std::cout << "Point has nearby points within search radius" << std::endl;
    }
    else
    {
        std::cout << "Point has no nearby points within search radius" << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/t3_cloud", 1, cloudCallback);

    ros::spin();
    return 0;
}

// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/transforms.h>
// #include <tf/transform_listener.h>

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "pcl_transform_example");
//     ros::NodeHandle nh;

//     // 创建一个tf监听器
//     tf::TransformListener listener;

//     // 创建一个发布器，用于发布转换后的点云数据
//     ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_pointcloud", 1);

//     // 创建一个订阅器，用于订阅雷达发布的点云数据
//     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("lidar_pointcloud", 1, [&](const sensor_msgs::PointCloud2ConstPtr& msg) {
//         // 将sensor_msgs/PointCloud2类型的数据转换成pcl::PointCloud<PointT>类型的数据
//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::fromROSMsg(*msg, *cloud);

//         // 将点云从相机坐标系转换到地图坐标系
//         tf::StampedTransform transform;
//         try{
//             // 获取相机坐标系到地图坐标系的变换
//             listener.lookupTransform("map", "base_link", ros::Time(0), transform);
//         }
//         catch (tf::TransformException ex){
//             ROS_ERROR("%s",ex.what());
//             return;
//         }

//         // 将点云进行坐标系变换
//         pcl_ros::transformPointCloud(*cloud, *cloud, transform);

//         // 将pcl::PointCloud<PointT>类型的数据转换成sensor_msgs/PointCloud2类型的数据
//         sensor_msgs::PointCloud2 output;
//         pcl::toROSMsg(*cloud, output);

//         // 发布转换后的点云数据
//         pub.publish(output);
//     });

//     ros::spin();
//     return 0;
// }

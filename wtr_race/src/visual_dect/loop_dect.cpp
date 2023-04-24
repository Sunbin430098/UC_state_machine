/**
 * @file task2.cpp
 * @author sunbin (you@domain.com)
 * @brief   读取摄像头的数据，转换成opencv处理后再以ROS节点发布
 *                  注意的一点就是图像可以从系统读取(opencv库)，或者ros订阅，根据自己需求选择获取方式
 *                  CmakeLists配置(fuck)，自己探索的，好tm折磨
 * @brief   处理的大逻辑总结一下：在原来的基础ROS编写逻辑下，创建的发布订阅者对象需要用image_transport的对象
 *                  对回调数据(基础ROS程序中的msg)，要通过cv_bridge转换为opencv能处理的bgr数据
 * @version 0.1
 * @date 2022-10-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <ros/ros.h>
//与python相同的三个包
#include <image_transport/image_transport.h>    //ROS和opencv数据转换
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//下面是C++文件需要的opencv库
#include <opencv2/imgproc/imgproc.hpp>                 //包含OpenCV的图像处理和GUI模块头文件
#include <opencv2/highgui/highgui.hpp>                     //比如打开关闭窗口  http://www.360doc.com/content/22/0424/14/79386973_1028062408.shtml
 

static const std::string OPENCV_WINDOW = "Image window";
 
class ImageConverter
{
    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        image_transport::Publisher image_pub_;
 
    public:
    ImageConverter() : it_(nh_)//构造函数
    {
        //接受和发布对象初始化不再是nh.subcribe
        //变成image_transport::ImageTreansport类型的对象
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);//订阅摄像头发布的话题
        image_pub_ = it_.advertise("/image_converter/output_video", 1);//发布者
 
        cv::namedWindow(OPENCV_WINDOW);     //自定义窗口命名
    }
 
    ~ImageConverter()//析构函数
    {
        cv::destroyWindow(OPENCV_WINDOW);   //关闭窗口
     }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
      //首先将ROS图像消息转换为了CvImage以在OpenCV中使用
      cv_bridge::CvImagePtr cv_ptr;

      //C++是try——catch组合
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//拷贝，bgr8字符
      }
      catch (cv_bridge::Exception& e)//&别名，相当于Python的as
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      //想在视频流中画一个圆
      if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        cv::circle(cv_ptr->image, cv::Point(50, 50), 60, CV_RGB(255,0,0),5);
  
      // 生成窗口
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(3);
  
      // 发布视频流
      image_pub_.publish(cv_ptr->toImageMsg());
    }
};
 
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
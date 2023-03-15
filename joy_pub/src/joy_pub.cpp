/**
 * @file joy_pub.cpp
 * @author sunbin (you@domain.com)
 * @brief 遥控器发送按钮指令
 * @version 0.1
 * @date 2022-10-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "joy_pub/joy_button.h"

class Joy_control
{
public:
  Joy_control();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);//私有的终端回调函数

  ros::NodeHandle nh_;

  int start_, stop_;
  ros::Subscriber joy_sub_;
  ros::ServiceClient model_client;

};

Joy_control::Joy_control():
  start_(1),
  stop_(0)
{
  nh_.param("start", start_, start_);
  nh_.param("stop", stop_, stop_);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Joy_control::joyCallback, this);

  model_client = nh_.serviceClient<joy_pub::joy_button>("model");

}

void Joy_control::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  
  joy_pub::joy_button model;
  if(joy->buttons[stop_]==1)
  {
    model.request.button_A=1;
    model.request.button_B=0;
    ROS_INFO("stop model");
  }
  else if(joy->buttons[start_]==1&&joy->buttons[stop_]==0)
  {
    model.request.button_A=0;
    model.request.button_B=1;
    ROS_INFO("start mdoel");
  }
  else{
    model.request.button_A=0;
    model.request.button_B=0;
  }
  ros::service::waitForService("model");
  bool flag = model_client.call(model);
  if (flag)
  {
    ROS_INFO("Call successfully: ");
  }
  else
  {
    ROS_ERROR("Ask error....");
  }
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "joy_node");
  Joy_control joy_control;
  ros::spin();
}
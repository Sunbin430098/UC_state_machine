/**
 * @file realsense.cpp
 * @author sunbin
 * @brief 手柄逻辑　Ｂ开启,Ａ紧急停止，再按一次继续运动
 * @version 0.1
 * @date 2022-10-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "joy_pub/up_machine.h"
#define PERIOD 50
#define Output_max 0.5

#define X_max 4
#define Y_max 1

#define A 1
#define lamda 2
#define PI 3.1415926

#define vx 1.0
#define vw 0

/**
 * @brief 定位里程计部分
 * 
 */
up_machine_ns::realsense_camera::realsense_camera()
{
    sub=nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample",1000,&up_machine_ns::realsense_camera::callback,this);
    target_path_pub = nh.advertise<nav_msgs::Path>("Target_path_pub",10);
    current_path_pub = nh.advertise<nav_msgs::Path>("Current_path_pub",10);
}
void up_machine_ns::realsense_camera::callback(const nav_msgs::Odometry::ConstPtr &msgs)
{
    // nav_msgs.header.frame_id = msgs->header.frame_id;
    // nav_msgs.header.seq = msgs->header.seq;
    // nav_msgs.header.stamp = msgs->header.stamp;
    // nav_msgs.child_frame_id = msgs->child_frame_id;

    nav_msgs.pose.pose.position.x = msgs->pose.pose.position.x;
    nav_msgs.pose.pose.position.y = msgs->pose.pose.position.y;

    nav_msgs.twist.twist.linear.x = msgs->twist.twist.linear.x;
    nav_msgs.twist.twist.linear.y = msgs->twist.twist.linear.y;
    nav_msgs.twist.twist.angular.z = msgs->twist.twist.angular.z;
    // std::cout << msgs->header.stamp << 'lalala ' << nav_msgs.pose.pose.position.x << ' ' << nav_msgs.pose.pose.position.y << ' ' << std::endl;
    nav_msgs::Path current_path;
    current_path.header.frame_id = "camera_odom_frame";
    current_path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped  current_pose;
    current_pose.pose.position.x = nav_msgs.pose.pose.position.x;
    current_pose.pose.position.y = nav_msgs.pose.pose.position.y;
    current_path.poses.push_back(current_pose);
    // current_path_pub.publish(current_path);
    // ROS_INFO("x=%f,y=%f",current_pose.pose.position.x,current_pose.pose.position.y);
    nav_msgs::Path target_path;
    current_path.header.frame_id = "camera_odom_frame";
    current_path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped  target_pose;
    target_pose.pose.position.x = nav_msgs.pose.pose.position.x;
    target_pose.pose.position.y = sin(target_pose.pose.position.x);
    target_path.poses.push_back(target_pose);
    // target_path_pub.publish(target_path);
}

/**
 * @brief 速度规划
 * 
 * @param position_3d 
 * @param speed_call 
 */
up_machine_ns::Speed_plan::Speed_plan():realsense_camera()
{
    joy_server = nh_.advertiseService("model",&up_machine_ns::Speed_plan::doReq,this);
    speed_pub = nh_.advertise<geometry_msgs::Twist>("/mavros/speed_control/send_speed",10);
    vel_thread = nh_.createTimer(ros::Duration(0.1), &up_machine_ns::Speed_plan::vel_callback,this);
    flag = false;
}

void up_machine_ns::Speed_plan::vel_callback(const ros::TimerEvent &e){
    up_machine_ns::Speed_plan::odom_plan(nav_msgs, v_pub);
}
void up_machine_ns::Speed_plan::odom_plan(nav_msgs::Odometry &nav_msgs ,geometry_msgs::Twist &speed_plan)
{
    float former;
    ROS_INFO("start count = %d",start_button_count);
    ROS_INFO("stop count = %d",stop_button_count);
    if(start_button_count%2==1)  
    {
        ROS_INFO("Start odom speed plan:");
        if((flag==false)&&(nav_msgs.pose.pose.position.x<1))
        {
            ROS_INFO("phase pre");
            speed_plan.linear.x = vx;
        }
        else if((flag==false)&&(nav_msgs.pose.pose.position.x <= X_max)&&(nav_msgs.pose.pose.position.x >= 1)&&(nav_msgs.pose.pose.position.x>=former)){
            ROS_INFO("phase forward");
            speed_plan.linear.x = vx;
            speed_plan.linear.y = vx * cos((nav_msgs.pose.pose.position.x-1)*2*PI/lamda); 
        }
        else if((flag==false)&&((nav_msgs.pose.pose.position.x) <= X_max+1)&&((nav_msgs.pose.pose.position.x)>=X_max)){
            ROS_INFO("phase back pre");
            speed_plan.linear.x = -1*vx;
            speed_plan.linear.y = 0;
            flag = true;
        }
        else if((flag == true)&&(nav_msgs.pose.pose.position.x<=X_max+1)&&(nav_msgs.pose.pose.position.x>=1))
        {
            ROS_INFO("phase back");
            speed_plan.linear.x = -1*vx;
            speed_plan.linear.y = -1*vx * cos(nav_msgs.pose.pose.position.x*2*PI/lamda);
            flag==true;
        }
        else{
            speed_plan.linear.x = 0;
            speed_plan.linear.y = 0;
        }
        // speed_plan.linear.y += up_machine_ns::Speed_plan::pidController(nav_msgs.pose.pose.position.y,nav_msgs.pose.pose.position.x,nav_msgs.twist.twist.linear.y);
        speed_plan.angular.z = vw;
        former = nav_msgs.pose.pose.position.x;
        ROS_INFO("pose_x=%f",nav_msgs.pose.pose.position.x);
    }
    if(stop_button_count%2==1){
        ROS_INFO("Stop condition");
        speed_plan.linear.x = 0;
        speed_plan.linear.y = 0;
        speed_plan.angular.z = 0;
    }
    ROS_INFO("vx=%f,vy=%f",speed_plan.linear.x,speed_plan.linear.y);
    // speed_plan.linear.y += up_machine_ns::Speed_plan::pidController(nav_msgs.pose.pose.position.y, nav_msgs.pose.pose.position.x,nav_msgs.twist.twist.linear.x);
    speed_pub.publish(speed_plan);
}
bool up_machine_ns::Speed_plan::doReq(joy_pub::joy_buttonRequest &req,joy_pub::joy_buttonResponse &resp)
{ 
    resp.send_success = 1;
    if(req.button_A==1)
    {
        ROS_INFO("model stop");
        stop_button_count ++;
    }
    else if(req.button_B==1)
    {
        ROS_INFO("model start");
        start_button_count ++;
    }
    else{
        resp.send_success = -1;
    }

    return resp.send_success;
}
float up_machine_ns::Speed_plan::pidController(float currentPosition_y, float currentPosition_x,float currentVelocity_y)
{
    float eI,eII,eIII;
    float output=0;

    if(flag==false){eI =  sin(currentPosition_x*2*PI/lamda)- currentPosition_y;}
    // else if (flag==true){eI =  -sin(currentPosition_x*2*PI/lamda)- currentPosition_y;}
    
    float Kp = 1, Ti = 100, Td = 0, T = 20;
    float q0 = Kp * (1 + T / Ti + Td / T);
    float q1 = -Kp * (1 + 2 * Td / T);
    float q2 = Kp * Td / T;
    output = q0 * eI + q1 * eII + q2 * eIII;
    eIII = eII;
    eII = eI;
    // output += eI;
    if (output >= Output_max)
    {
        output = Output_max;  
    }
    if (output <= -Output_max)
    {
        output = -Output_max; 
    }
    
    return output;
}

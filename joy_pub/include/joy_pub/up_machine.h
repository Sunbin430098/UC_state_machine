#ifndef _UP_MACHINE_H
#define _UP_MACHINE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/Twist.h"

#include "joy_pub/joy_button.h"
#include "joy_pub/speed_plan.h"
#include "joy_pub/speed_call.h"

#include <math.h>

namespace up_machine_ns
{
    class realsense_camera 
    {
        public:
            realsense_camera();
            nav_msgs::Odometry nav_msgs;
        private:
            ros::NodeHandle nh;
            ros::Subscriber sub;
            ros::Publisher target_path_pub;
            ros::Publisher current_path_pub;
            void callback(const nav_msgs::Odometry::ConstPtr &msgs);

    };
    class Speed_plan:public realsense_camera
    {
        public:
            Speed_plan();
            void odom_plan(nav_msgs::Odometry &nav_msgs ,geometry_msgs::Twist &speed_call);
            float pidController(float targetPosition_y, float currentPosition_x, float currentVelocity_y);
        private:
            bool doReq(joy_pub::joy_buttonRequest &req,joy_pub::joy_buttonResponse &resp);
            ros::NodeHandle nh_;
            ros::ServiceServer joy_server;
            ros::Publisher speed_pub;
            ros::Timer vel_thread;
            void vel_callback(const ros::TimerEvent &e);
            geometry_msgs::Twist v_pub;
            int32_t stop_button_count;
            int32_t start_button_count;
            bool flag ;
    };
}

#endif

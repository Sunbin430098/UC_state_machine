/**
 * @brief 码盘、速度数据通信
 * @author sunbin
 *
 * @addtogroup plugin
 */
 
#include <mavros/mavros_plugin.h>
#include <mavros_msgs/Speed.h>
#include <mavros_msgs/Disk.h>

#include "geometry_msgs/Twist.h"

#define scale 0.001

namespace mavros {
namespace std_plugins {
class SpeedControlPlugin : public plugin::PluginBase {
private:
	// void cb(const mavros_msgs::Speed::ConstPtr &speed_msg)
	void cb(const geometry_msgs::Twist::ConstPtr &speed_msg)
	{
		ROS_INFO("Mavros receive successfully:");
		// mavlink::common::msg::V_SET v_set;
		mavlink::common::msg::SPEED_CONTROL_SET v_set;
		v_set.vx_set = speed_msg->linear.x;
		v_set.vy_set = speed_msg->linear.y;
		v_set.vw_set = speed_msg->angular.z;
		//调用mavlink消息发送API
		ROS_INFO("vx=%f,vy=%f,vz=%f",v_set.vx_set,v_set.vy_set,v_set.vw_set);
		UAS_FCU(m_uas)->send_message_ignore_drop(v_set);
	}
public:
	SpeedControlPlugin() : PluginBase(),
		speed_control_nh("~speed_control")
	{ }
 
	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		disk_pub = speed_control_nh.advertise<mavros_msgs::Disk>("motion_handle", 10);
		// speed_sub = speed_control_nh.subscribe<mavros_msgs::Speed>("motion_command",10,&SpeedControlPlugin::cb,this);
		 speed_sub = speed_control_nh.subscribe<geometry_msgs::Twist>("motion_command",10,&SpeedControlPlugin::cb,this);

	}
    //用来获取mavlink解析到的消息
	Subscriptions get_subscriptions() {
		ROS_INFO("HIHI");
		return {
			make_handler(&SpeedControlPlugin::handle_disk),
		};
	}

private:
	ros::NodeHandle speed_control_nh;
 
	ros::Publisher disk_pub;
	ros::Subscriber speed_sub;
	ros::ServiceServer send_service;
 
	/* -*- rx handlers -*- */
	void handle_disk(const mavlink::mavlink_message_t *msg, mavlink::common::msg::DISK_DATA &disk_msg)
	{
		auto disk_data = boost::make_shared<mavros_msgs::Disk>();
		ROS_INFO("HAHA");
		disk_data->x = disk_msg.x * scale;
		disk_data->y = disk_msg.y * scale;
		disk_data->w = disk_msg.w * scale;
		disk_data->yaw = disk_msg.yaw;
		disk_data->pitch = disk_msg.pitch;
		disk_data->roll = disk_msg.roll;
		//将解析到的消息发布至topic
		disk_pub.publish(disk_data);
		ROS_INFO("I receive:%f,%f,%f,%f,%f,%f",disk_data->x ,disk_data->y,disk_data->w,disk_data->yaw,disk_data->pitch,disk_data->roll);
	}
};
}	// namespace std_plugins
}	// namespace mavros
 
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SpeedControlPlugin, mavros::plugin::PluginBase)
 
/**
 * @brief SpeedControls plugin 插件功能主要为实现mavlink数据包与mavros自定义消息之间的转化
 * @file speed_control.cpp
 * @addtogroup plugin
 */

#include <mavros/mavros_plugin.h>
#include "mavlink/v2.0/common/mavlink_msg_posture.hpp"
#include "mavlink/v2.0/common/mavlink_msg_control.hpp"
#include "mavros_msgs/wtr_control.h"
#include "mavros_msgs/wtr_posture.h"

#include "geometry_msgs/Twist.h"


namespace mavros {
namespace std_plugins {
/**
 * @brief Speed Control plugin
 */
class SpeedControlPlugin : public plugin::PluginBase {
public:
	SpeedControlPlugin() : PluginBase(),
		speed_control_nh("~speed_control")
	{ }
	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		control_pub = speed_control_nh.advertise<mavros_msgs::wtr_control>("status", 10);
		// send_service = speed_control_nh.advertiseService("send", &SpeedControlPlugin::send_cb_service, this);
		// send_subscriber = speed_control_nh.subscribe<mavros_msgs::SpeedControlSet_sub>("send_topic", 10, &SpeedControlPlugin::send_callback_subscribe, this);
		send_subscriber = speed_control_nh.subscribe<geometry_msgs::Twist>("send_topic", 10, &SpeedControlPlugin::send_callback_subscribe, this);
		
		pos_publisher = speed_control_nh.advertise<mavros_msgs::wtr_posture>("posture", 10);
	
	}

    /**
     * @brief Get the subscriptions object
	 *        用来获取mavlink消息
	 * 		  获取到mavlink包后进入handle_speed_control进行解析
     * 
     * @return Subscriptions 
     */
	Subscriptions get_subscriptions() {
		ROS_INFO("get_subscriptions success!");
		return {
			make_handler(&SpeedControlPlugin::handle_speed_control),
		};
	}

private:
	ros::NodeHandle speed_control_nh;
	ros::Publisher control_pub;
	ros::ServiceServer send_service;
	ros::Subscriber send_subscriber;
	ros::Publisher pos_publisher;
 
	/**
	 * @brief rx handlers 接收到mavlink包后调用此函数，将mavlink数据包解析为mavros中的自定义消息，并发布到话题
	 * 
	 * @attention common::msg::SPEED_CONTROL_STATUS为自动生成的消息头文件中所定义的，也是依据此来解析收到的mavlink消息
	 * @param msg 
	 * @param posture_state 
	 */
	
	void handle_speed_control(const mavlink::mavlink_message_t *msg, mavlink::common::msg::POSTURE &posture_state)
	{
		auto posture_state_msg = boost::make_shared<mavros_msgs::wtr_posture>();

		speed_control_nh.setParam("A1",posture_state.A1);
		speed_control_nh.setParam("A2",posture_state.A2);
		speed_control_nh.setParam("A3",posture_state.A3);
		speed_control_nh.setParam("B1",posture_state.B1);
		speed_control_nh.setParam("B2",posture_state.B2);
		speed_control_nh.setParam("B3",posture_state.B3);		
		speed_control_nh.setParam("C1",posture_state.C1);
		speed_control_nh.setParam("C2",posture_state.C2);
		speed_control_nh.setParam("C3",posture_state.C3);
		speed_control_nh.setParam("C4",posture_state.C4);
		speed_control_nh.setParam("D1",posture_state.D1);		
	}
 
	/**
	 * @brief callbacks 服务端或者订阅者获得消息后进入callback，将消息转化为mavlink数据包，通过mavlink消息发送API，发送给下位机
	 * 
	 * @param req 
	 * @param responce 
	 * @return true 
	 * @return false 
	 */
 
	// bool send_cb_service(mavros_msgs::SpeedControlSet::Request &req , mavros_msgs::SpeedControlSet::Response &responce)
	// {
	// 	mavlink::common::msg::CONTROL_SET msg;
	// 	//将server收到的request赋值给mavlink消息
	// 	msg.vx_set = req.vx_set;
	// 	msg.vy_set = req.vy_set;
	// 	msg.vw_set = req.vw_set;
		
	// 	//响应发送成功
	// 	responce.send_success = true;
		
	// 	//调用mavlink消息发送API
	// 	UAS_FCU(m_uas)->send_message_ignore_drop(msg);
	// 	return true;
	// }

	void send_callback_subscribe(const geometry_msgs::Twist::ConstPtr& speed_p)
	{
		mavlink::common::msg::CONTROL msg;
		msg.vw_set = speed_p->angular.z;
		msg.vy_set = speed_p->linear.y;
		msg.vx_set = speed_p->linear.x;
		// msg.vw_set = 1;
		// msg.vy_set = 2;
		// msg.vx_set = 3;
		ROS_INFO("send_callback succcess!");
		ROS_INFO("vx=%f,vy=%f,vw=%f",msg.vx_set,msg.vy_set,msg.vw_set);
		UAS_FCU(m_uas)->send_message_ignore_drop(msg);
	} 
	
};
}	// namespace std_plugins
}	// namespace mavros
 
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SpeedControlPlugin, mavros::plugin::PluginBase)
 
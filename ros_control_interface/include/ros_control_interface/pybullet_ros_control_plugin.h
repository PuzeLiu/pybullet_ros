#ifndef ROS_CONTROL_INTERFACE_PYBULLET_ROS_CONTROL_PLUGIN_H_
#define ROS_CONTROL_INTERFACE_PYBULLET_ROS_CONTROL_PLUGIN_H_

#include "ros/ros.h"
#include <std_msgs/Bool.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <transmission_interface/transmission_parser.h>

namespace ros_control_interface
{
	class PybulletRosControlPlugin
	{
	 public:
		PybulletRosControlPlugin() = default;

		~PybulletRosControlPlugin() = default;

		bool Load(std::string model_ns, std::string env_ns);

		std::string robot_namespace_;

	 protected:
		void eStopCB(const std_msgs::BoolConstPtr& e_stop_active);

		// Node Handles
		ros::NodeHandle model_nh_; // namespaces to robot name

		// Strings
		std::string robot_description_;

		// Transmissions in this plugin's scope
		std::vector<transmission_interface::TransmissionInfo> transmissions_;

		ros::Duration control_period_;

		// e_stop_active_ is true if the emergency stop is active.
		bool e_stop_active_, last_e_stop_active_;
		ros::Subscriber e_stop_sub_;  // Emergency stop subscriber
	};
}

#endif //ROS_CONTROL_INTERFACE_PYBULLET_ROS_CONTROL_PLUGIN_H_

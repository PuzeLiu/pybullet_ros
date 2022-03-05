#ifndef ROS_CONTROL_INTERFACE_PYBULLET_ROS_CONTROL_PLUGIN_H_
#define ROS_CONTROL_INTERFACE_PYBULLET_ROS_CONTROL_PLUGIN_H_

#include "ros/ros.h"
#include <std_msgs/Bool.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <transmission_interface/transmission_parser.h>

#include "default_robot_hw_sim.h"

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

        // Interface loader
        boost::shared_ptr<pluginlib::ClassLoader<ros_control_interface::RobotHWSim> > robot_hw_sim_loader_;
        void load_robot_hw_sim_srv();

		// Strings
		std::string robot_description_;

		// Transmissions in this plugin's scope
		std::vector<transmission_interface::TransmissionInfo> transmissions_;

        // Robot simulator interface
        std::string robot_hw_sim_type_str_;
        boost::shared_ptr<ros_control_interface::RobotHWSim> robot_hw_sim_;

        // Controller manager
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

        // Timing
		ros::Duration control_period_;
        ros::Time last_update_sim_time_ros_;
        ros::Time last_write_sim_time_ros_;

		// e_stop_active_ is true if the emergency stop is active.
		bool e_stop_active_, last_e_stop_active_;
		ros::Subscriber e_stop_sub_;  // Emergency stop subscriber
	};
}

#endif //ROS_CONTROL_INTERFACE_PYBULLET_ROS_CONTROL_PLUGIN_H_

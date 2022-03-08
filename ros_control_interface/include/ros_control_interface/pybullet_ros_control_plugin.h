/*
 * MIT License
 * Copyright (c) 2022 Puze Liu
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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

		~PybulletRosControlPlugin()
		{
			robot_hw_sim_loader_->unloadLibraryForClass(robot_hw_sim_type_str_);
		};

		bool Load(std::string model_ns, std::string env_ns);

		std::vector<double> Update(double time, std::vector<double> &joint_pos, std::vector<double> &joint_vel,
			std::vector<double> &joint_effort);

		void Unload();

		std::string robot_namespace_;
		// Interface loader
		boost::shared_ptr<pluginlib::ClassLoader<ros_control_interface::RobotHWSim> > robot_hw_sim_loader_;
		boost::shared_ptr<ros_control_interface::RobotHWSim> robot_hw_sim_;

	 protected:
		void eStopCB(const std_msgs::BoolConstPtr& e_stop_active);

		// Node Handles
		ros::NodeHandle model_nh_; // namespaces to robot name

		// Strings
		std::string robot_description_;

		// Transmissions in this plugin's scope
		std::vector<transmission_interface::TransmissionInfo> transmissions_;

        // Robot simulator interface
        std::string robot_hw_sim_type_str_;

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

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

#ifndef ROS_CONTROL_INTERFACE_DEFAULT_ROBOT_HW_SIM_H_
#define ROS_CONTROL_INTERFACE_DEFAULT_ROBOT_HW_SIM_H_

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

#include <boost/algorithm/clamp.hpp>

// ros_control_interface
#include "robot_hw_sim.h"

namespace ros_control_interface{
	class DefaultRobotHWSim : public ros_control_interface::RobotHWSim
	{
	 public:
		virtual bool initSim(
			const std::string& robot_namespace,
			ros::NodeHandle model_nh,
			const urdf::Model *const urdf_model,
			std::vector<transmission_interface::TransmissionInfo> transmissions);

		virtual void readSim(ros::Time time, ros::Duration period, std::vector<double> joint_position,
			std::vector<double> joint_velocity, std::vector<double> joint_effort);

		virtual void writeSim(ros::Time time, ros::Duration period);

		virtual void eStopActive(const bool active);

	 protected:

		// Register the limits of the joint specified by joint_name and joint_handle. The limits are
		// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
		// Return the joint's type, lower position limit, upper position limit, and effort limit.
		void registerJointLimits(const std::string& joint_name,
			const hardware_interface::JointHandle& joint_handle,
			const ControlMethod ctrl_method,
			const ros::NodeHandle& joint_limit_nh,
			const urdf::Model *const urdf_model,
			int *const joint_type, double *const lower_limit,
			double *const upper_limit, double *const effort_limit);

		unsigned int n_dof_;

		hardware_interface::JointStateInterface    js_interface_;
		hardware_interface::EffortJointInterface   ej_interface_;
		hardware_interface::PositionJointInterface pj_interface_;
		hardware_interface::VelocityJointInterface vj_interface_;

		joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
		joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
		joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
		joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
		joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
		joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;

		std::vector<control_toolbox::Pid> pid_controllers_;

		// e_stop_active_ is true if the emergency stop is active.
		bool e_stop_active_, last_e_stop_active_;
	};

}


#endif //ROS_CONTROL_INTERFACE_DEFAULT_ROBOT_HW_SIM_H_

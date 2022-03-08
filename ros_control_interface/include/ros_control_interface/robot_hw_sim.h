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

#ifndef ROS_CONTROL_INTERFACE_ROBOT_HW_SIM_H_
#define ROS_CONTROL_INTERFACE_ROBOT_HW_SIM_H_

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/transmission_info.h>
#include <urdf/model.h>

namespace ros_control_interface{

	// Methods used to control a joint.
	enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};

	class RobotHWSim : public hardware_interface::RobotHW
	{
	 public:

		virtual ~RobotHWSim() { }

		/// \brief Initialize the simulated robot hardware
		///
		/// Initialize the simulated robot hardware.
		///
		/// \param robot_namespace  Robot namespace.
		/// \param model_nh  Model node handle.
		/// \param parent_model  Parent model.
		/// \param urdf_model  URDF model.
		/// \param transmissions  Transmissions.
		///
		/// \return  \c true if the simulated robot hardware is initialized successfully, \c false if not.
		virtual bool initSim(
			const std::string& robot_namespace,
			ros::NodeHandle model_nh,
			const urdf::Model *const urdf_model,
			std::vector<transmission_interface::TransmissionInfo> transmissions) = 0;

		/// \brief Read state data from the simulated robot hardware
		///
		/// Read state data, such as joint positions and velocities, from the simulated robot hardware.
		///
		/// \param time  Simulation time.
		/// \param period  Time since the last simulation step.
		virtual void readSim(ros::Time time, ros::Duration period, std::vector<double> joint_position,
			std::vector<double> joint_velocity, std::vector<double> joint_effort) = 0;

		/// \brief Write commands to the simulated robot hardware
		///
		/// Write commands, such as joint position and velocity commands, to the simulated robot hardware.
		///
		/// \param time  Simulation time.
		/// \param period  Time since the last simulation step.
		virtual void writeSim(ros::Time time, ros::Duration period) = 0;

		/// \brief Set the emergency stop state
		///
		/// Set the simulated robot's emergency stop state. The default implementation of this function does nothing.
		///
		/// \param active  \c true if the emergency stop is active, \c false if not.
		virtual void eStopActive(const bool active) {}

	 public:
		std::vector<std::string> joint_names_;
		std::vector<int> joint_types_;
		std::vector<double> joint_lower_limits_;
		std::vector<double> joint_upper_limits_;
		std::vector<double> joint_effort_limits_;
		std::vector<ControlMethod> joint_control_methods_;
		std::vector<double> joint_position_;
		std::vector<double> joint_velocity_;
		std::vector<double> joint_effort_;
		std::vector<double> joint_effort_command_;
		std::vector<double> joint_position_command_;
		std::vector<double> last_joint_position_command_;
		std::vector<double> joint_velocity_command_;
		std::vector<double> joint_command_out_;

	};

}

#endif //ROS_CONTROL_INTERFACE_ROBOT_HW_SIM_H_

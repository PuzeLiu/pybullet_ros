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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>

#include "ros_control_interface/pybullet_ros_control_plugin.h"

namespace py = pybind11;
namespace rci = ros_control_interface;

PYBIND11_MODULE(pybullet_ros_control_plugin_py, m){
	m.doc() = "Python bindings of the hardware interface for pybullet";
	py::class_<rci::PybulletRosControlPlugin>(m, "PybulletRosControlPlugin")
	    .def(py::init<>())
		.def_readwrite("robot_namespace_", &rci::PybulletRosControlPlugin::robot_namespace_)
		.def("get_joint_names", [](rci::PybulletRosControlPlugin *self){
		  return self->robot_hw_sim_->joint_names_;
		})
		.def("get_joint_effort_limits", [](rci::PybulletRosControlPlugin *self){
		  return self->robot_hw_sim_->joint_effort_limits_;
		})
		.def("get_joint_control_methods", [](rci::PybulletRosControlPlugin *self){
		  return self->robot_hw_sim_->joint_control_methods_;
		})
		.def("load", [](rci::PybulletRosControlPlugin *self, std::string robot_ns,	std::string env_ns = "pybullet_ros"){
			return self->Load(robot_ns, env_ns);
		}, py::arg("robot_ns"), py::arg("env_ns")="pybullet_ros")
		.def("update", [](rci::PybulletRosControlPlugin *self, double time, std::vector<double> joint_position,
			std::vector<double> joint_velocity, std::vector<double> joint_effort){
			self->Update(time, joint_position, joint_velocity, joint_effort);
			  return self->robot_hw_sim_->joint_command_out_;
			},	py::arg("sim_time"), py::arg("joint_position"),
				py::arg("joint_velocity"), py::arg("joint_effort"))
		.def("unload", &rci::PybulletRosControlPlugin::Unload)
		;

	py::enum_<rci::ControlMethod>(m, "ControlMethod")
	    .value("EFFORT", rci::ControlMethod::EFFORT)
		.value("POSITION", rci::ControlMethod::POSITION)
		.value("POSITION_PID", rci::ControlMethod::POSITION_PID)
		.value("VELOCITY", rci::ControlMethod::VELOCITY)
		.value("VELOCITY_PID", rci::ControlMethod::VELOCITY_PID);

}
#include <pybind11/pybind11.h>

#include <Eigen/Dense>

#include "ros_control_interface/pybullet_ros_control_plugin.h"

namespace py = pybind11;
namespace rci = ros_control_interface;

PYBIND11_MODULE(pybullet_ros_control_plugin_py, m){
	m.doc() = "Python bindings of the hardware interface for pybullet";
	py::class_<rci::PybulletRosControlPlugin>(m, "PybulletRosControlPlugin")
	    .def(py::init<>())
		.def("load", [](rci::PybulletRosControlPlugin *self, std::string robot_ns,
			std::string env_ns = "pybullet_ros"){
			return self->Load(robot_ns, env_ns);
		}, py::arg("robot_ns"), py::arg("env_ns")="pybullet_ros")
		.def_readwrite("robot_namespace_", &rci::PybulletRosControlPlugin::robot_namespace_);
}
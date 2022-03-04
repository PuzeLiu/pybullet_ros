#include "ros/ros.h"
#include "urdf/model.h"
#include "ros_control_interface/pybullet_ros_control_plugin.h"

namespace ros_control_interface
{
	bool PybulletRosControlPlugin::Load(std::string model_ns, std::string env_ns)
	{
		ros::NodeHandle pybullet_nh(env_ns);
		robot_namespace_ = model_ns;
		model_nh_ = ros::NodeHandle(robot_namespace_);

		if (!model_nh_.getParam("robot_description", robot_description_))
		{
			ROS_ERROR_STREAM("Unable to find " << model_nh_.getNamespace() << "/robot_description");
			return false;
		}
		urdf::Model urdf_model;
		urdf_model.initString(robot_description_);

		// Decide the plugin control period
		ros::Duration pybullet_period(1 / pybullet_nh.param("loop_rate", 100.));
		double control_period_tmp;
		if (model_nh_.getParam("control_period", control_period_tmp))
		{
			control_period_ = ros::Duration(control_period_tmp);
			// Check the period against the simulation period
			if (control_period_ < pybullet_period)
			{
				ROS_ERROR_STREAM_NAMED("gazebo_ros_control", "Desired controller update period ("
					<< control_period_ << " s) is faster than the gazebo simulation period (" << pybullet_period
					<< " s).");
				return false;
			}
			else if (control_period_ > pybullet_period)
			{
				ROS_WARN_STREAM_NAMED("gazebo_ros_control", "Desired controller update period ("
					<< control_period_ << " s) is slower than the gazebo simulation period (" << pybullet_period
					<< " s).");
			}
		}
		else
		{
			control_period_ = pybullet_period;
		}

		// Initialize the emergency stop code.
		e_stop_active_ = false;
		last_e_stop_active_ = false;
		std::string e_stop_topic;
		if (model_nh_.getParam("eStopTopic", e_stop_topic))
		{
			e_stop_sub_ = model_nh_.subscribe(e_stop_topic, 1, &PybulletRosControlPlugin::eStopCB, this);
		}

		ROS_INFO_NAMED("pybullet_ros_control", "Starting pybullet_ros_control plugin in namespace: %s", robot_namespace_.c_str());

		if (!transmission_interface::TransmissionParser::parse(robot_description_, transmissions_)){
			ROS_ERROR_NAMED("pybullet_ros_control", "Error parsing URDF in pybullet_ros_control plugin, plugin not active.\n");
			return false;
		}



		return true;
	}

	// Emergency stop callback
	void PybulletRosControlPlugin::eStopCB(const std_msgs::BoolConstPtr& e_stop_active)
	{
		e_stop_active_ = e_stop_active->data;
	}
}
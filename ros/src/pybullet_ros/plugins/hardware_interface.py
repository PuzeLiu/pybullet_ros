from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init

from pybullet_ros.pybullet_ros_control_plugin_py import PybulletRosControlPlugin


class HardwareInterface:
    def __init__(self, pybullet, ns, model_spec, **kwargs):
        roscpp_init("test", [])
        self.plugin_interface = PybulletRosControlPlugin()
        self.plugin_interface.load(ns)

    def execute(self, time):
        pass

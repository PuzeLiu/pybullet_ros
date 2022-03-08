import numpy as np
import rospy
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init

import pybullet_ros.pybullet_ros_control_plugin_py as pbrc


class HardwareInterface:
    def __init__(self, pybullet, ns, model_spec, **kwargs):
        roscpp_init("test", [])
        self.pb_client = pybullet
        self.model_id = model_spec['model_id']

        self.plugin_interface = pbrc.PybulletRosControlPlugin()
        self.plugin_interface.load(ns)
        self.joint_names = self.plugin_interface.get_joint_names()
        self.joint_effort_limits = self.plugin_interface.get_joint_effort_limits()

        # Find the pybullet joint index in sequence
        self.pb_joint_idx = list()
        self.pb_joint_control_mode = list()
        ros_control_modes = self.plugin_interface.get_joint_control_methods()
        for i, joint_name in enumerate(self.joint_names):
            self.pb_joint_idx.append(model_spec['joint_map'][joint_name][1])
            if ros_control_modes[i] == pbrc.ControlMethod.POSITION:
                self.pb_joint_control_mode.append(self.pb_client.POSITION_CONTROL)
            elif ros_control_modes[i] == pbrc.ControlMethod.VELOCITY:
                self.pb_joint_control_mode.append(self.pb_client.VELOCITY_CONTROL)
                self.pb_client.setJointMotorControl2(*model_spec['joint_map'][joint_name],
                                                     controlMode=self.pb_client.VELOCITY_CONTROL, force=0)
            elif ros_control_modes[i] == pbrc.ControlMethod.EFFORT or \
                    ros_control_modes[i] == pbrc.ControlMethod.POSITION_PID or \
                    ros_control_modes[i] == pbrc.ControlMethod.VELOCITY_PID:
                self.pb_joint_control_mode.append(self.pb_client.TORQUE_CONTROL)
                self.pb_client.setJointMotorControl2(*model_spec['joint_map'][joint_name],
                                                     controlMode=self.pb_client.VELOCITY_CONTROL, force=0)
            else:
                raise rospy.INFO("Unknown Control Mode for joint: ", joint_name)

    def execute(self, time):
        joint_states = np.array(self.pb_client.getJointStates(self.model_id, self.pb_joint_idx), dtype=object)
        pos = joint_states[:, 0].astype(float)
        vel = joint_states[:, 1].astype(float)
        torque = joint_states[:, 3].astype(float)
        cmd = self.plugin_interface.update(time.to_sec(), joint_position=pos,
                                           joint_velocity=vel, joint_effort=torque)

        for i, idx in enumerate(self.pb_joint_idx):
            if self.pb_joint_control_mode[i] == self.pb_client.POSITION_CONTROL:
                self.pb_client.setJointMotorControl2(self.model_id, idx,
                                                     self.pb_joint_control_mode[i],
                                                     targetPosition=cmd[i],
                                                     force=self.joint_effort_limits[i])
            elif self.pb_joint_control_mode[i] == self.pb_client.VELOCITY_CONTROL:
                self.pb_client.setJointMotorControl2(self.model_id, idx,
                                                     self.pb_joint_control_mode[i],
                                                     targetVelocity=cmd[i],
                                                     force=self.joint_effort_limits[i])
            elif self.pb_joint_control_mode[i] == self.pb_client.TORQUE_CONTROL:
                self.pb_client.setJointMotorControl2(self.model_id, idx,
                                                     self.pb_joint_control_mode[i],
                                                     force=cmd[i])

    def __del__(self):
        self.plugin_interface.unload()

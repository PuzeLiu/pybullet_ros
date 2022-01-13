#!/usr/bin/env python3

"""
query robot state and publish position, velocity and effort values to /joint_states
"""
import time

import rospy
from sensor_msgs.msg import JointState


class JoinStatePub:
    def __init__(self, pybullet, namespace, model_spec, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        self.namespace = namespace
        self.model_spec = model_spec
        # register this node in the network as a publisher in /joint_states topic
        self.pub_joint_states = rospy.Publisher(namespace + '/joint_states', JointState, queue_size=1)
        self.period = rospy.Duration(1 / kargs['publish_rate'])
        self.joint_msg = JointState()
        for name in self.model_spec['joint_map']:
            self.joint_msg.name.append(name)

        self.next_up_time = rospy.Time.now() + self.period

    def execute(self, sim_time):
        if (sim_time - self.next_up_time).to_sec() > -1e-8:
            """this function gets called from pybullet ros main update loop"""
            self.joint_msg.position.clear()
            self.joint_msg.velocity.clear()
            self.joint_msg.effort.clear()
            # get joint states
            for name, joint_index in self.model_spec['joint_map'].items():

                # get joint state from pybullet
                joint_state = self.pb.getJointState(*joint_index)
                # fill msg
                self.joint_msg.position.append(joint_state[0])
                self.joint_msg.velocity.append(joint_state[1])
                self.joint_msg.effort.append(joint_state[3]) # applied effort in last sim step
            # update msg time using ROS time api
            self.joint_msg.header.stamp = sim_time
            # publish joint states to ROS
            self.pub_joint_states.publish(self.joint_msg)
            self.next_up_time = sim_time + self.period

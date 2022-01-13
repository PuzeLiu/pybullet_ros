#!/usr/bin/env python3

import os
import re
import importlib
import time
import rospy
import rospkg
import pybullet_data

from std_srvs.srv import Empty
from rosgraph_msgs.msg import Clock
from pybullet_ros.function_exec_manager import FuncExecManager

class pyBulletRosWrapper(object):
    """ROS wrapper class for pybullet simulator"""
    def __init__(self):
        # import pybullet
        self.pb = importlib.import_module('pybullet')
        # get from param server the frequency at which to run the simulation
        self.loop_rate = rospy.get_param('~loop_rate', 80.0)
        # query from param server if gui is needed
        is_gui_needed = rospy.get_param('~pybullet_gui', True)
        # get from param server if user wants to pause simulation at startup
        self.pause_simulation = rospy.get_param('~pause_simulation', False)

        print('\033[34m')
        # print pybullet stuff in blue
        physicsClient = self.start_gui(gui=is_gui_needed)
        rospy.set_param('~client', physicsClient)
        # get pybullet path in your system and store it internally for future use, e.g. to set floor
        self.pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        phsics_param = rospy.get_param('~physics', {})
        print("Physics Parameter", phsics_param)
        self.pb.setPhysicsEngineParameter(**phsics_param)

        # set /use_sim_time in the parameter server
        if not rospy.has_param('/use_sim_time'):
            rospy.set_param('/use_sim_time', True)
        self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=10)
        self.sim_time = 0.
        self.time_step = 1 / self.loop_rate
        self.clock_msg = Clock()

        # setup service to restart simulation
        rospy.Service('~reset_simulation', Empty, self.handle_reset_simulation)
        # setup services for pausing/unpausing simulation
        rospy.Service('~pause_physics', Empty, self.handle_pause_physics)
        rospy.Service('~unpause_physics', Empty, self.handle_unpause_physics)

        # create object of environment class for later use
        env_plugin = rospy.get_param('~environment', 'environment')  # default : plugins/environment.py
        self.environment = getattr(importlib.import_module(f'{env_plugin}'), 'Environment')(self.pb)
        # load environment
        rospy.loginfo('loading environment')
        self.environment.load_environment()

        # create models in pybullet
        self.models = {}
        self.plugins = []
        for ns in rospy.get_param('~models', ['']):
            if ns != "":
                model_spec = dict()
                model_spec['model_id'] = self.init_model(ns)
                model_spec['joint_map'] = self.get_joint_map(model_spec['model_id'])
                rospy.loginfo("loading plugin for {}: ".format(ns))
                model_plugins = rospy.get_param(ns + "/plugins", [])

                for plugin in model_plugins:
                    module_ = plugin.pop("module")
                    class_ = plugin.pop("class")
                    params_ = plugin.copy()
                    rospy.loginfo('loading plugin: {} class from {}'.format(class_, module_))
                    # create object of the imported file class
                    obj = getattr(importlib.import_module(module_), class_)(self.pb, ns, model_spec, **params_)
                    # obj.execute()
                    self.plugins.append(obj)

                self.models[ns] = model_spec
        print('\033[0m')
        rospy.loginfo('pybullet ROS wrapper initialized')

    def get_joint_map(self, model_id):
        joint_map = {}
        for joint_id in range(0, self.pb.getNumJoints(model_id)):
            info = self.pb.getJointInfo(model_id, joint_id)

            if info[2] != self.pb.JOINT_FIXED:
                joint_name = info[1].decode('utf-8')
                joint_map[joint_name] = (model_id, joint_id)
        return joint_map

    def handle_reset_simulation(self, req):
        """Callback to handle the service offered by this node to reset the simulation"""
        rospy.loginfo('reseting simulation now')
        self.pb.resetSimulation()
        return Empty()

    def start_gui(self, gui=True):
        """start physics engine (client) with or without gui"""
        if(gui):
            # start simulation with gui
            rospy.loginfo('Running pybullet with gui')
            rospy.loginfo('-------------------------')
            gui_options = rospy.get_param('~gui_options', '')  # e.g. options="--width=2560 --height=1440"
            return self.pb.connect(self.pb.GUI_SERVER, options=gui_options)
        else:
            # start simulation without gui (non-graphical version)
            rospy.loginfo('Running pybullet without gui')
            # hide console output from pybullet
            rospy.loginfo('-------------------------')
            return self.pb.connect(self.pb.SHARED_MEMORY)

    def init_model(self, namespace):
        """load robot URDF model, set gravity, ground plane and environment"""
        # get from param server the path to the URDF robot model to load at startup
        robot_description = rospy.get_param(namespace + '/robot_description', None)
        if not robot_description:
            rospy.logerr('{} required /robot_description param not set'.format(namespace))
            return None

        # redirect the $(find package)
        package = re.search('package://(.+?)/', robot_description)
        if package:
            package_dir = rospkg.RosPack().get_path(package.group(1))
            robot_description = robot_description.replace("package://" + package.group(1), package_dir)

        urdf_dir = os.path.abspath(os.path.dirname(__file__) + "../../../.urdf")
        if not os.path.exists(urdf_dir):
            os.makedirs(urdf_dir)
        rospy.loginfo('generating urdf model for {} from robot_description under: {}'.format(namespace, urdf_dir))
        urdf_file = os.path.join(urdf_dir, namespace + ".urdf")
        file = open(urdf_file, 'w')
        file.write(robot_description)
        file.close()

        # load robot from URDF model
        # user decides if inertia is computed automatically by pybullet or custom
        urdf_flags = 0
        if rospy.get_param(namespace + '/use_inertia_from_file', False):
            # combining several boolean flags using "or" according to pybullet documentation
            urdf_flags = self.pb.URDF_USE_INERTIA_FROM_FILE
            rospy.loginfo("{} use inertia from file".format(namespace))
        return self.pb.loadURDF(urdf_file, flags=urdf_flags)

    def handle_reset_simulation(self, req):
        """Callback to handle the service offered by this node to reset the simulation"""
        rospy.loginfo('reseting simulation now!')
        # pause simulation to prevent reading joint values with an empty world
        self.pause_simulation = True
        # remove all objects from the world and reset the world to initial conditions
        self.pb.resetSimulation()
        # load URDF model again, set gravity and floor
        for ns in rospy.get_param('~models', ['']):
            if ns != "":
                model_spec = dict()
                model_spec['model_id'] = self.init_model(ns)
        # resume simulation control cycle now that a new robot is in place
        self.pause_simulation = False
        # reset time
        self.sim_time = 0.
        return []

    def handle_pause_physics(self, req):
        """pause simulation, raise flag to prevent pybullet to execute self.pb.stepSimulation()"""
        rospy.loginfo('pausing simulation')
        self.pause_simulation = False
        return []

    def handle_unpause_physics(self, req):
        """unpause simulation, lower flag to allow pybullet to execute self.pb.stepSimulation()"""
        rospy.loginfo('unpausing simulation')
        self.pause_simulation = True
        return []

    def pause_simulation_function(self):
        return self.pause_simulation

    def step_with_time(self):
        self.pb.stepSimulation()
        self.sim_time += self.time_step
        self.clock_msg.clock = rospy.Time.from_sec(self.sim_time)
        self.clock_pub.publish(self.clock_msg)

    def start_pybullet_ros_wrapper_sequential(self):
        """
        This function is deprecated, we recommend the use of parallel plugin execution
        """
        time_start = time.time()
        while not rospy.is_shutdown():
            if not self.pause_simulation:
                self.step_with_time()
                # run x plugins
                for task in self.plugins:
                    task.execute(rospy.Time.from_sec(self.sim_time))
                # perform all the actions in a single forward dynamics simulation step such
                # as collision detection, constraint solving and integration
            # rate.sleep()
            time.sleep(max(self.sim_time - (time.time() - time_start), 0))
        rospy.logwarn('killing node now...')
        # if node is killed, disconnect
        self.pb.disconnect()

    def start_pybullet_ros_wrapper_parallel(self):
        """
        Execute plugins in parallel, however watch their execution time and warn if exceeds the deadline (loop rate)
        """
        # create object of our parallel execution manager
        exec_manager_obj = FuncExecManager(self.plugins, rospy.is_shutdown, self.step_with_time, self.pause_simulation_function,
                                    log_info=rospy.loginfo, log_warn=rospy.logwarn, log_debug=rospy.logdebug, function_name='plugin')
        # start parallel execution of all "execute" class methods in a synchronous way
        exec_manager_obj.start_synchronous_execution(loop_rate=self.loop_rate)
        # ctrl + c was pressed, exit
        rospy.logwarn('killing node now...')
        # if node is killed, disconnect
        self.pb.disconnect()

    def start_pybullet_ros_wrapper(self):
        if rospy.get_param('~parallel_plugin_execution', True):
            self.start_pybullet_ros_wrapper_parallel()
        else:
            self.start_pybullet_ros_wrapper_sequential()


def main():
    """function called by pybullet_ros_node script"""
    rospy.init_node('pybullet_ros', anonymous=False)  # node name gets overwritten if launched by a launch file
    pybullet_ros_interface = pyBulletRosWrapper()
    pybullet_ros_interface.start_pybullet_ros_wrapper()


if __name__ == '__main__':
    main()

import rclpy

from geometry_msgs.msg import Twist
from .submodule.config import *



class RobotisOP3Driver:



    def init(self, webots_node, properties):
        # Get webots parameters
        self.__robot    = webots_node.robot
        self.__joint    = {}

        # Get joints
        for joint_name in ROBOT_CONFIG_YAML['robot_info'][ROBOTIS_OP3_ENUM]['webots_joints']:
            self.__joint[joint_name] = self.__robot.getDevice(joint_name)
            self.__joint[joint_name].setPosition(float('inf'))
            self.__joint[joint_name].setVelocity(0)

        # Initiate rclpy node
        rclpy.init(args=None)
        self.node = rclpy.create_node(properties['rclnode'])



    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        self.__joint['ShoulderR'].setVelocity(0.2)

import rclpy
import std_msgs.msg as std_msgs

from rclpy.node import Node



# DoboticsPybulletRobotNode Class
# This class is made for interfacing the robot.
class DoboticsPybulletRobotNode(Node):



    def __init__(self, robot_name:str) -> None:
        # Initiate the node
        super().__init__('dobotics/' + robot_name)

        # Read config/robot/robot_config.yaml file


        # Initiate the publishers and subscribers
import rclpy
import std_msgs.msg as std_msgs

from rclpy.node import Node
from .config import *



# DoboticsPybulletRobotNode Class
# This class is made for interfacing the Robotis OP3 robot.
class DoboticsPybulletOP3(Node):



    def __init__(self) -> None:
        # Initiate the node
        super().__init__('default')

        # Initiate the publishers and subscribers
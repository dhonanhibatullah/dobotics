import rclpy
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs
import numpy as np
import cv2

from rclpy.node import Node
from cv_bridge import CvBridge



class DoboticsRobotisOP3Controller(Node):



    def __init__(self):
        # Create ROS2 node instance
        super().__init__('default')

        # Retrieve the parameter
        self.declare_parameter('robot_name_param', rclpy.Parameter.Type.STRING)
        
        # Variables/attributes
        self.camera_image   = None
        self.robot_name     = self.get_parameter('robot_name_param').get_parameter_value().string_value
        self.img_bridge     = CvBridge() # For webots -> openCV image conversion purposes

        # Initiate publishers and subscribers
        self.create_subscription(sensor_msgs.Image, f'/{self.robot_name}/Camera/image_color', self.cameraCB, 10)



    # Callback methods
    def cameraCB(self, msg):
        # Convert ROS Image message to OpenCV image
        self.camera_image = self.img_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')



def main(args=None):
    # Initiate ROS2 node
    rclpy.init(args=args)
    robot_camera = DoboticsRobotisOP3Controller()

    # Loop
    while rclpy.ok():

        # spin once the callbacks
        rclpy.spin_once(robot_camera)
    
    # Terminate
    robot_camera.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
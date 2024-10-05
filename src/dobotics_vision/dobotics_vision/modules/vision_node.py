import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .profiles.BallDetectionV1 import *


class VisionNode(Node):

    def __init__(self) -> None:
        super().__init__('VisionNode')

        self.declare_parameter('robotName', rclpy.Parameter.Type.STRING)
        self.robot_name = self.get_parameter('robotName').value
        self.camera_image = None
        self.cv_bridge = CvBridge()

        self.camera_sub = self.create_subscription(
            msg_type=Image,
            topic=f'{self.robot_name}/Camera/image_color',
            callback=self.cameraSubCallback,
            qos_profile=10
        )
        self.vision_pub = self.create_publisher(
            msg_type=Image,
            topic=f'{self.robot_name}/vision',
            qos_profile=10
        )


    def cameraSubCallback(self, msg:Image) -> None:
        self.camera_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgra8')
        ball_detect_res = BallDetectionV1(self.camera_image)

        img_pub = cv2.cvtColor(ball_detect_res['image'], cv2.COLOR_GRAY2BGR)
        self.vision_pub.publish(self.cv_bridge.cv2_to_imgmsg(img_pub, 'bgra8'))
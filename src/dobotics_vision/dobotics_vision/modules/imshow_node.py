import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge


class ImshowNode(Node):

    def __init__(self) -> None:
        super().__init__('ImshowNode')
        self.declare_parameter('row', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('col', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('topics', rclpy.Parameter.Type.STRING_ARRAY)

        self.row = self.get_parameter('row').value
        self.col = self.get_parameter('col').value
        self.topics = self.get_parameter('topics').value

        self.get_logger().info(f'{self.topics}')

        self.bridge = CvBridge()
        self.scale_factor = 1./float(self.row) if self.row > self.col else 1./float(self.col)
        self.topics_num = len(self.topics)

        self.subs = {}
        self.imgs = {}
        self.update_sts = {}

        for topic in self.topics:
            self.subs.update({
                topic: self.create_subscription(
                    msg_type=Image,
                    topic=topic,
                    callback=lambda msg,topic=topic: self.imageSubsCallback(msg, topic),
                    qos_profile=10
                )
            })
            self.imgs.update({topic: None})
            self.update_sts.update({topic: False})


    def imageSubsCallback(self, msg: Image, topic: str) -> None:
        self.update_sts[topic] = True
        self.imgs[topic] = self.addTextToImage(
            self.uniformScale(
                self.bridge.imgmsg_to_cv2(msg), 
                self.scale_factor
            ), 
            topic
        )

        is_completed = True
        for topic in self.topics:
            if not self.update_sts[topic]: is_completed = False

        if is_completed:
            for topic in self.topics: self.update_sts[topic] = False

            res_img = self.concatenateImages()
            cv2.imshow('Dobotics - Imshow', res_img)
            cv2.waitKey(1)


    def uniformScale(self, img: np.ndarray, scale: float) -> np.ndarray:
        height, width = img.shape[:2]
        return cv2.resize(img, (int(width*scale), int(height*scale)), interpolation=cv2.INTER_LINEAR)
    

    def addTextToImage(self, img: np.ndarray, text: str):
        height, width = img.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.4
        color = (255, 255, 255)
        thickness = 1
        text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
        text_x = (width - text_size[0]) // 2
        text_y = 15
        
        new_image = cv2.copyMakeBorder(img, 20, 0, 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))
        cv2.putText(new_image, text, (text_x, text_y), font, font_scale, color, thickness, cv2.LINE_AA)
        
        return new_image
    

    def concatenateImages(self) -> np.ndarray:
        topic_cnt = 0
        terminate = False

        col_img = []
        for col in range(self.col):
            
            row_img = []
            for row in range(self.row):
                row_img.append(self.imgs[self.topics[topic_cnt]])
                topic_cnt += 1
                if topic_cnt >= self.topics_num: 
                    terminate = True
                    break

            col_img.append(cv2.hconcat(row_img))
            if terminate: break

        return cv2.vconcat(col_img)
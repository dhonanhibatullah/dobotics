import rclpy
from .modules.vision_node import VisionNode


def main(args=None) -> None:
    rclpy.init()
    node = VisionNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
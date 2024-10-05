import rclpy
from .modules.imshow_node import ImshowNode


def main(args=None) -> None:
    rclpy.init()
    node = ImshowNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
import std_msgs.msg as std_msgs
import controller as webots


class SupervisorDriver:

    def init(self, webots_node, properties) -> None:
        self.robot: webots.Robot            = webots_node.robot
        self.reset_position: list[float]    = properties['resetPosition']
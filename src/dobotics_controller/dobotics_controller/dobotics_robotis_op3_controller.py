import rclpy

from .submodule.dobotics_robotis_op3_controller_node import DoboticsRobotisOP3Controller



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
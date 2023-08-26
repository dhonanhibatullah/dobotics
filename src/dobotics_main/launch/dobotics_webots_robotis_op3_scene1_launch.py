import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    # Get the robot .urdf
    PKG_PATH        = get_package_share_directory('dobotics_main')
    ROBOT_DESC_PATH = os.path.join(PKG_PATH, 'resource', 'robotis_op3.urdf')

    # Launch webots
    webots = WebotsLauncher(
        world       = os.path.join(PKG_PATH, 'worlds', 'robotis_op3_scene1.wbt')
    )

    # Launch the world with controller
    robot_controller = WebotsController(
        robot_name  = 'robotis_op3',
        parameters  = [
            {'robot_description': ROBOT_DESC_PATH},
        ]
    )

    return LaunchDescription([
        webots,
        robot_controller,
        launch.actions.RegisterEventHandler(
            event_handler = launch.event_handlers.OnProcessExit(
                target_action   = webots,
                on_exit         = [launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
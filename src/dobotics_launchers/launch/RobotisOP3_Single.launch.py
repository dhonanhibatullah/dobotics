import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    WEBOTS_SHARE_PATH           = get_package_share_directory('dobotics_webots')
    SUPERVISOR_URDF_PATH        = os.path.join(WEBOTS_SHARE_PATH, 'projects', 'RobotisOP3', 'descriptions', 'single_supervisor.urdf')
    SINGLE_URDF_PATH            = os.path.join(WEBOTS_SHARE_PATH, 'projects', 'RobotisOP3', 'descriptions', 'single.urdf')
    WORLD_PATH                  = os.path.join(WEBOTS_SHARE_PATH, 'projects', 'RobotisOP3', 'worlds', 'single.wbt')
    
    webots = WebotsLauncher(world = WORLD_PATH)

    robotis_op3_driver = WebotsController(
        robot_name  = 'robotis_op3',
        parameters  = [
            {'robot_description': SINGLE_URDF_PATH},
        ]
    )

    return LaunchDescription([
        webots,
        robotis_op3_driver,
        launch.actions.RegisterEventHandler(
            event_handler = launch.event_handlers.OnProcessExit(
                target_action   = webots,
                on_exit         = [launch.actions.EmitEvent(event = launch.events.Shutdown())],
            )
        )
    ])
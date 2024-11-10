import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    WEBOTS_SHARE_PATH       = get_package_share_directory('dobotics_webots')
    SUPERVISOR_URDF_PATH    = os.path.join(WEBOTS_SHARE_PATH, 'project', 'RobotisOP3', 'descriptions', 'match_1v1_supervisor.urdf')
    ROBOT_A_URDF_PATH       = os.path.join(WEBOTS_SHARE_PATH, 'project', 'RobotisOP3', 'descriptions', 'match_1v1_a.urdf')
    ROBOT_B_URDF_PATH       = os.path.join(WEBOTS_SHARE_PATH, 'project', 'RobotisOP3', 'descriptions', 'match_1v1_b.urdf')
    WORLD_PATH              = os.path.join(WEBOTS_SHARE_PATH, 'project', 'RobotisOP3', 'worlds', 'match_1v1.wbt')
    
    webots = WebotsLauncher(world = WORLD_PATH)

    supervisor_driver = WebotsController(
        robot_name  = 'supervisor',
        parameters  = [
            {'robot_description': SUPERVISOR_URDF_PATH}
        ]
    )

    robotis_op3_a_driver = WebotsController(
        robot_name  = 'robotis_op3_a',
        parameters  = [
            {'robot_description': ROBOT_A_URDF_PATH},
        ]
    )

    robotis_op3_b_driver = WebotsController(
        robot_name  = 'robotis_op3_b',
        parameters  = [
            {'robot_description': ROBOT_B_URDF_PATH},
        ]
    )

    return LaunchDescription([
        webots,
        supervisor_driver,
        robotis_op3_a_driver,
        robotis_op3_b_driver,
        launch.actions.RegisterEventHandler(
            event_handler = launch.event_handlers.OnProcessExit(
                target_action   = webots,
                on_exit         = [launch.actions.EmitEvent(event = launch.events.Shutdown())],
            )
        )
    ])
import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    PKG_PATH = get_package_share_directory('dobotics_main')
    ROBOT_A_URDF = os.path.join(PKG_PATH, 'resource', 'robotis_op3_a.urdf')
    ROBOT_B_URDF = os.path.join(PKG_PATH, 'resource', 'robotis_op3_b.urdf')
    webots = WebotsLauncher(world=os.path.join(PKG_PATH, 'worlds', 'robotis_op3_single_match.wbt'))

    robotis_op3_a_driver = WebotsController(
        robot_name='robotis_op3_a',
        parameters=[
            {'robot_description': ROBOT_A_URDF},
        ]
    )

    robotis_op3_a_vision = Node(
        package='dobotics_vision',
        executable='vision',
        name='robotis_op3_a_vision',
        parameters=[
            {'robotName': 'robotis_op3_a'}
        ]
    )

    robotis_op3_b_driver = WebotsController(
        robot_name='robotis_op3_b',
        parameters=[
            {'robot_description': ROBOT_B_URDF},
        ]
    )

    robotis_op3_b_vision = Node(
        package='dobotics_vision',
        executable='vision',
        name='robotis_op3_b_vision',
        parameters=[
            {'robotName': 'robotis_op3_b'}
        ]
    )

    return LaunchDescription([
        webots,
        robotis_op3_a_driver,
        robotis_op3_a_vision,
        robotis_op3_b_driver,
        robotis_op3_b_vision,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
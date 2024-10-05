import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    PKG_PATH = get_package_share_directory('dobotics_main')
    ROBOT_URDF = os.path.join(PKG_PATH, 'resource', 'robotis_op3_single.urdf')
    webots = WebotsLauncher(world=os.path.join(PKG_PATH, 'worlds', 'robotis_op3_single.wbt'))

    robotis_op3_driver = WebotsController(
        robot_name='robotis_op3',
        parameters=[
            {'robot_description': ROBOT_URDF},
        ]
    )

    robotis_op3_vision = Node(
        package='dobotics_vision',
        executable='vision',
        name='robotis_op3_vision',
        parameters=[
            {'robotName': 'robotis_op3'}
        ]
    )

    return LaunchDescription([
        webots,
        robotis_op3_driver,
        robotis_op3_vision,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
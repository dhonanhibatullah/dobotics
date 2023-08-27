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
    ROBOT_DESC_PATH = os.path.join(PKG_PATH, 'resource')

    # Launch webots
    webots = WebotsLauncher(
        world       = os.path.join(PKG_PATH, 'worlds', 'robotis_op3_scene2.wbt')
    )

    # Launch the world with controller
    teamA_striker_driver = WebotsController(
        robot_name  = 'robotis_op3_teamA_striker',
        parameters  = [
            {'robot_description': ROBOT_DESC_PATH + '/robotis_op3_teamA_striker.urdf'},
        ]
    )
    teamA_striker_controller = Node(
        package     = 'dobotics_controller',
        executable  = 'dobotics_robotis_op3_controller',
        name        = 'robotis_op3_teamA_striker_controller',
        parameters  = [
            {'robot_name_param': 'robotis_op3_teamA_striker'}
        ]
    )

    teamA_keeper_driver = WebotsController(
        robot_name  = 'robotis_op3_teamA_keeper',
        parameters  = [
            {'robot_description': ROBOT_DESC_PATH + '/robotis_op3_teamA_keeper.urdf'},
        ]
    )
    teamA_keeper_controller = Node(
        package     = 'dobotics_controller',
        executable  = 'dobotics_robotis_op3_controller',
        name        = 'robotis_op3_teamA_keeper_controller',
        parameters  = [
            {'robot_name_param': 'robotis_op3_teamA_keeper'}
        ]
    )

    teamB_striker_driver = WebotsController(
        robot_name  = 'robotis_op3_teamB_striker',
        parameters  = [
            {'robot_description': ROBOT_DESC_PATH + '/robotis_op3_teamB_striker.urdf'},
        ]
    )
    teamB_striker_controller = Node(
        package     = 'dobotics_controller',
        executable  = 'dobotics_robotis_op3_controller',
        name        = 'robotis_op3_teamB_striker_controller',
        parameters  = [
            {'robot_name_param': 'robotis_op3_teamB_striker'}
        ]
    )

    teamB_keeper_driver = WebotsController(
        robot_name  = 'robotis_op3_teamB_keeper',
        parameters  = [
            {'robot_description': ROBOT_DESC_PATH + '/robotis_op3_teamB_keeper.urdf'},
        ]
    )
    teamB_keeper_controller = Node(
        package     = 'dobotics_controller',
        executable  = 'dobotics_robotis_op3_controller',
        name        = 'robotis_op3_teamB_keeper_controller',
        parameters  = [
            {'robot_name_param': 'robotis_op3_teamB_keeper'}
        ]
    )

    # Return the launch desc.
    return LaunchDescription([
        webots,
        teamA_striker_driver,
        teamA_striker_controller,
        teamA_keeper_driver,
        teamA_keeper_controller,
        teamB_striker_driver,
        teamB_striker_controller,
        teamB_keeper_driver,
        teamB_keeper_controller,
        launch.actions.RegisterEventHandler(
            event_handler = launch.event_handlers.OnProcessExit(
                target_action   = webots,
                on_exit         = [launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
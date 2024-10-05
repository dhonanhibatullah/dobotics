import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


IMSHOW_CONFIG = yaml.safe_load(open(os.path.join(os.getcwd(), 'src', 'dobotics_main', 'config', 'imshow.yaml'), 'r'))


def generate_launch_description():

    imshow_node = Node(
        package='dobotics_vision',
        executable='imshow',
        name='imshow_node',
        parameters=[
            {'row': IMSHOW_CONFIG['row']},
            {'col': IMSHOW_CONFIG['col']},
            {'topics': IMSHOW_CONFIG['image_topics']}
        ]
    )

    return LaunchDescription([imshow_node])
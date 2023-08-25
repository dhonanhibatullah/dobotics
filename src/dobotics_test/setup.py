import os
from glob import glob
from setuptools import setup

package_name    = 'dobotics_test'
submodule       = 'dobotics_test/submodule'
access_test     = 'dobotics_test/submodule/access_test'

setup(
    name                = package_name,
    version             = '0.0.0',
    packages            = [package_name, submodule, access_test],
    data_files          = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py'))
    ],
    install_requires    = ['setuptools'],
    zip_safe            = True,
    maintainer          = 'dhonan',
    maintainer_email    = 'dhonan.hibatullah@gmail.com',
    description         = 'This package made for running several tests on the dobotics workspace.',
    license             = 'None',
    tests_require       = ['pytest'],
    entry_points        = {
        'console_scripts': [
            'ros2_talker_example = dobotics_test.ros2_talker_example:main',
            'ros2_listener_example = dobotics_test.ros2_listener_example:main',
            'yaml_test = dobotics_test.yaml_test:main',
            'directory_structure_test = dobotics_test.directory_structure_test:main'
        ],
    },
)
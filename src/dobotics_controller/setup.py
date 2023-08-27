import os
from glob import glob
from setuptools import setup

package_name                    = 'dobotics_controller'
submodule                       = 'dobotics_controller/submodule'
robotis_op3_controller          = 'dobotics_controller/submodule/robotis_op3_controller'
robotis_turtlebot3_controller   = 'dobotics_controller/submodule/robotis_turtlebot3_controller'

setup(
    name                = package_name,
    version             = '0.0.0',
    packages            = [
        package_name,
        submodule,
        robotis_op3_controller,
        robotis_turtlebot3_controller
    ],
    data_files          = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py'))
    ],
    install_requires    = ['setuptools'],
    zip_safe            = True,
    maintainer          = 'dhonan',
    maintainer_email    = 'dhonan.hibatullah@gmail.com',
    description         = 'This package contains robot controllers.',
    license             = 'None',
    tests_require       = ['pytest'],
    entry_points        = {
        'console_scripts': [
            'dobotics_robotis_op3_controller = dobotics_controller.dobotics_robotis_op3_controller:main'
        ],
    },
)

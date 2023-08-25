import os
from glob import glob
from setuptools import setup

package_name    = 'dobotics_main'
submodule       = 'dobotics_main/submodule'

setup(
    name                = package_name,
    version             = '0.0.0',
    packages            = [package_name, submodule],
    data_files          = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py'))
    ],
    install_requires    = ['setuptools'],
    zip_safe            = True,
    maintainer          = 'dhonan',
    maintainer_email    = 'dhonan.hibatullah@gmail.com',
    description         = 'This package is made for launching the main nodes. Most of the workspace features can be ran from dobotics_main',
    license             = 'None',
    tests_require       = ['pytest'],
    entry_points        = {
        'console_scripts': [
            'dobotics_pybullet_sim = dobotics_main.dobotics_pybullet_sim:main'
        ],
    },
)

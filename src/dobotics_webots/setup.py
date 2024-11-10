import os
from glob import glob
from setuptools import setup

package_name = 'dobotics_webots'
package_list = [
    package_name,
]
project_list = [
    'RobotisOP3',
    'ClearpathPR2'
]
driver_list = [
    'supervisor',
    'robotis_op3'
]

def generate_projects() -> list[tuple]:
    projects = []
    for project_name in project_list:
        projects.append(('share/' + package_name + '/project' + f'/{project_name}' + '/worlds', glob(f'project/{project_name}/worlds/*.wbt')))
        projects.append(('share/' + package_name + '/project' + f'/{project_name}' + '/descriptions', glob(f'project/{project_name}/descriptions/*.urdf')))
        projects.append(('share/' + package_name + '/project' + f'/{project_name}' + '/params', glob(f'project/{project_name}/params/*.param.json')))
    return projects

def generate_console_script() -> list[str]:
    scripts = []
    for driver_name in driver_list:
        scripts.append(f'{driver_name} = dobotics_webots.{driver_name}:main')
    return scripts


setup(
    name                = package_name,
    version             = '0.0.0',
    packages            = package_list,
    data_files          = generate_projects() + [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires    = ['setuptools'],
    zip_safe            = True,
    maintainer          = 'dhonan',
    maintainer_email    = 'dhonan.hibatullah@gmail.com',
    description         = 'None',
    license             = 'None',
    tests_require       = ['pytest'],
    entry_points        = {
        'console_scripts': generate_console_script()
    },
)
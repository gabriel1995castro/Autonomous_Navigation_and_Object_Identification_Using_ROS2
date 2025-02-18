from setuptools import find_packages, setup
import os
from glob import glob 

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/Worlds', glob('Worlds/*.world')), 
        ('share/' + package_name + '/launch', [
            'launch/robot_controller_launch.py',
            'launch/world_A_launch.py',
            'launch/world_B_launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriel',
    maintainer_email='gabriel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = robot_controller.object_detector_node:main',
            'robot_nav6 = robot_controller.robot_nav6:main',
        ],
    },
)


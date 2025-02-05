import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_controller',
            executable='object_detector',
            name='object_detector_node',
            output='screen'
        ),
        Node(
            package='robot_controller',
            executable='robot_navigation',
            name='robot_navigation_node',
            output='screen'
        )
    ])

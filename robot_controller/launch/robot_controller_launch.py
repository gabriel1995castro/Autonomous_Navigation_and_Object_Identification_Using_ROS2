import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch/online_async_launch.py'
            )
        )
    )

    return LaunchDescription([
        slam_toolbox_launch,
        
	Node(
	    package='robot_controller',
	    executable='object_detector',
	    name='object_detector_node',
	    output='screen'
	),
        
    Node(
            package='robot_controller',
            executable='robot_nav6',
            name='robot_navigation_node',
            output='screen'
        ),
        
    ])


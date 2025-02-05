import os
import random
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    x_pose = str(random.uniform(-1.0, 1.0))
    y_pose = str(random.uniform(-1.0, 1.0))
    yaw_angle = str(random.uniform(-3.14, 3.14))  
    
   
    pkg_share = get_package_share_directory('turtlebot3_gazebo')

    return LaunchDescription([
        DeclareLaunchArgument('x_pose', default_value=x_pose, description='Posição X inicial'),
        DeclareLaunchArgument('y_pose', default_value=y_pose, description='Posição Y inicial'),
        DeclareLaunchArgument('yaw', default_value=yaw_angle, description='Rotação inicial'),

        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3',
                '-x', LaunchConfiguration('x_pose'),
                '-y', LaunchConfiguration('y_pose'),
                '-Y', LaunchConfiguration('yaw'),
                '-file', os.path.join(pkg_share, 'models', 'turtlebot3_waffle', 'model.sdf')
            ],
            output='screen'
        )
    ])

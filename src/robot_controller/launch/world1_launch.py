import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Caminho fixo para o arquivo do mundo
    world_path = '/home/gabriel/ros2_ws/src/robot_controller/Worlds/world1.world'

    # Declaração de argumentos configuráveis
    debug_arg = DeclareLaunchArgument('debug', default_value='false', description='Modo debug')
    gui_arg = DeclareLaunchArgument('gui', default_value='true', description='Exibir interface gráfica')
    pause_arg = DeclareLaunchArgument('pause', default_value='false', description='Iniciar simulação pausada')

    # Incluir o launch do Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('gazebo_ros').find('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'debug': LaunchConfiguration('debug'),
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('pause'),
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        debug_arg,
        gui_arg,
        pause_arg,
        gazebo_launch
    ])


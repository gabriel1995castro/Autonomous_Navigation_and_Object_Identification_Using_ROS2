import os
import random
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Obtém o diretório compartilhado do pacote
    package_share_dir = get_package_share_directory('robot_controller')

    # Caminho relativo para o arquivo do mundo
    world_path = os.path.join(package_share_dir, 'Worlds', 'world_B.world')

    # Argumentos configuráveis
    debug_arg = DeclareLaunchArgument('debug', default_value='false', description='Modo debug')
    gui_arg = DeclareLaunchArgument('gui', default_value='true', description='Exibir interface gráfica')
    pause_arg = DeclareLaunchArgument('pause', default_value='false', description='Iniciar simulação pausada')

    # Incluir o launch do Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'debug': LaunchConfiguration('debug'),
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('pause'),
            'use_sim_time': 'true'
        }.items()
    )

    # Gerar posição aleatória do robô
    x_pose = str(random.uniform(-1.0, 1.0))
    y_pose = str(random.uniform(-1.0, 1.0))
    yaw_angle = str(random.uniform(-3.14, 3.14))  

    # Diretório do modelo do TurtleBot3
    pkg_share = get_package_share_directory('turtlebot3_gazebo')

    # Nó para spawnar o robô
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3',
            '-x', x_pose,
            '-y', y_pose,
            '-Y', yaw_angle,
            '-file', os.path.join(pkg_share, 'models', 'turtlebot3_burger', 'model.sdf')
        ],
        output='screen'
    )

    return LaunchDescription([
        debug_arg,
        gui_arg,
        pause_arg,
        gazebo_launch,
        spawn_robot
    ])



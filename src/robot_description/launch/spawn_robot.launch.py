import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # 1. Dosya Yollarını Tanımla
    pkg_name = 'robot_description'
    file_subpath = 'urdf/my_waffle.urdf.xacro' # custom turtlebot robotumun urdfi 

    # Paketin tam yolunu bul
    pkg_path = get_package_share_directory(pkg_name)
    
    # Xacro dosyasının tam yolu
    xacro_file = os.path.join(pkg_path, file_subpath)

    # 2. Xacro'yu İşle (XML formatına çevir)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 3. Gazebo'yu Başlat (Boş bir dünya ile)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 4. Robot State Publisher Node'u
    # Bu node, robotun modelini topic olarak yayınlar (/robot_description)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': True}]
    )

    # 5. Robotu Gazebo'ya Ekleme (Spawn) Node'u
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_waffle_bot'], # Robotun Gazebo'daki ismi
        output='screen'
    )
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller"],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )


    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        diff_drive_spawner,
        joint_state_spawner,
        ])
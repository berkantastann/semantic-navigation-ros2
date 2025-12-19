import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_name = 'robot_description'
    file_subpath = 'urdf/full.xacro' # custom turtlebot robotumun urdfi 

    pkg_path = get_package_share_directory(pkg_name)

    xacro_file = os.path.join(pkg_path, file_subpath)

    robot_description_raw = xacro.process_file(xacro_file).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': True}]
    )

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
        ])
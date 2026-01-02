import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'robot_description'
    pkg_share = get_package_share_directory(pkg_name)

    map_file_path = os.path.join(pkg_share, 'maps', 'tugbot_warehouse_slam_map_edited.yaml')

    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file_path,
            'use_sim_time': 'true',
            'params_file': os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml'),
            'autostart': 'true'
        }.items()
    )
  
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Nav2 (/cmd_vel) -> Controller (/diff_drive_base_controller/cmd_vel_unstamped)

    cmd_vel_bridge = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel', '/diff_drive_base_controller/cmd_vel_unstamped'],
        output='screen'
    )

    return LaunchDescription([
        spawn_robot_cmd,
        nav2_bringup_cmd,
        rviz_cmd,
        cmd_vel_bridge
       
    ])
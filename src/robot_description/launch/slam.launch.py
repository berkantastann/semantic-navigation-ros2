import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_name = 'robot_description'
    config_file = 'config/mapper_params_online_async.yaml'
    config_path = os.path.join(get_package_share_directory(pkg_name), config_file)

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock'
    )

    # SLAM Toolbox Node'u
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            config_path,
            {'use_sim_time': use_sim_time}
        ]
    )

    rviz_config_path = os.path.join(
        get_package_share_directory(pkg_name), 'config', "slam_config_file.rviz"
    )
    
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )
        
        
    return LaunchDescription([
        declare_use_sim_time,
        slam_node,
        rviz_node
    ])
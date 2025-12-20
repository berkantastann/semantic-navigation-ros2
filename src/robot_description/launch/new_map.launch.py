import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_name = 'robot_description'
    file_subpath = 'urdf/my_waffle.urdf.xacro'

    pkg_path = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_path, file_subpath)
    
    ekf_config_path = os.path.join(pkg_path, 'config', 'ekf.yaml')

    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    world_file_path = os.path.join(pkg_path, 'worlds', 'tugbot_warehouse.sdf')

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),
                          'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_file_path}'}.items(),
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_waffle_bot',
                   '-x', '13.0',
                   '-y', '-10.0',
                   '-z', '0.2'], 
        output='screen'
    )

    # KÖPRÜ (BRIDGE)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        output='screen'
    )

    # TF Düzeltmeleri
    lidar_tf_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_fix',
        arguments=['0', '0', '0', '0', '0', '0', 'base_scan', 'my_waffle_bot/base_footprint/hls_lfcd_lds'],
        output='screen'
    )

    camera_tf_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_fix',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_rgb_optical_frame', 'my_waffle_bot/base_footprint/camera'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller", "--controller-manager-timeout", "30"],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "30"],
    )

    # EKF NODE (robot_localization) 
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path,       
            {'use_sim_time': True}  
        ]
    )

    return LaunchDescription([
        ign_gazebo,
        node_robot_state_publisher,
        spawn_entity,
        bridge,
        lidar_tf_fix,  
        camera_tf_fix, 
        diff_drive_spawner,
        joint_state_spawner,
        ekf_node, 
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # RTAB-Map parametreleri
    parameters = [{
        'frame_id': 'base_footprint',
        'subscribe_depth': True,
        'subscribe_scan': True,
        'approx_sync': True,          
        'use_action_for_goal': True,
        'qos_scan': 2,
        'qos_image': 2,
        'qos_imu': 2,
        
        # VO kapalı
        'visual_odometry': False,     
        
        #Ekf topic
        'odom_topic': '/odometry/filtered', 
        
        # RTAB-Map Tuning
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.01',
        'RGBD/LinearUpdate': '0.01',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'Grid/FromDepth': 'false', # Lidar varken haritayı derinlikten üretmeye gerek yok (Lidar daha temiz)
        
     
        'Reg/Strategy': '1',       
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.1'
    }]

    remappings = [
        ('rgb/image', '/camera/image'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('depth/image', '/camera/depth_image'),
        ('scan', '/scan'),
        ('odom', '/odometry/filtered') # EKF verisini odom olarak içeri alıyoruz - Ben sensor fusion ile odometry/filtered topic'ini oluşturuyorum
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # SLAM Modu (Haritalama)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            # Veritabanını silme işini bu argüman yapıyor
            arguments=['--delete_db_on_start'] 
        ),

        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=parameters,
            remappings=remappings
        )
    ])
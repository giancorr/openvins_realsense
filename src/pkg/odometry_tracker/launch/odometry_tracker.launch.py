from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # EKF parameters file path
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("openvins_bringup"), "config", "ekf.yaml"]
    )

    return LaunchDescription([
        # 1. Converter: IMU odometry to base_link
        Node(
            package='odometry_tracker',
            executable='odom_to_baselink',
            name='odom_to_baselink',
            output='screen'
        ),
        
        # 2. EKF Node from robot_localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),
        
        # 3. Path Publisher
        Node(
            package='odometry_tracker',
            executable='base_link_path_pub',
            name='base_link_path_pub',
            output='screen',
            parameters=[{
                'odom_topic': '/odometry/filtered'
            }]
        ),
        
        # 4. Simple VIO Bridge to PX4
        Node(
            package='odometry_tracker',
            executable='simple_vio_bridge',
            name='simple_vio_bridge',
            output='screen'
        )
    ])

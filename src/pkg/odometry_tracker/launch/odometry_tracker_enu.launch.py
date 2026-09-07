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
        # 1. Converter: IMU odometry to base_link (ENU Version)
        Node(
            package='odometry_tracker',
            executable='odom_to_baselink_enu',
            name='odom_to_baselink_enu',
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
        
        # 3. Path Publisher (for visualization)
        Node(
            package='odometry_tracker',
            executable='base_link_path_pub',
            name='base_link_path_pub',
            output='screen',
            parameters=[{
                'odom_topic': '/odometry/filtered'
            }]
        ),
        
        # 4. PX4 TF Publisher & VIO Bridge
        # This node takes /odometry/filtered (in ENU) and relays it to /fmu/in/vehicle_visual_odometry (in NED)
        Node(
            package='drone_odometry',
            executable='px4_tf_pub',
            name='px4_tf_pub',
            output='screen',
            parameters=[
                {'px4_odom_frame_id': 'odom'},
                {'vio_desired_parent_frame_id': 'odom'},
                {'publish_tf': False}, # EKF already publishes odom -> base_link
                {'is_already_ned': False} # Our input from EKF is ENU, let px4_tf_pub convert it to NED
            ]
        )
    ])

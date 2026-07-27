import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('openvins_bringup')

    # --- Front instance (cam0 + cam0 IMU) ---
    front_config = os.path.join(bringup_dir, 'config', 'estimator_config_front.yaml')
    front_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        name='ov_msckf',
        namespace='front',
        output='screen',
        parameters=[
            {"verbosity": "INFO"},
            {"use_stereo": False},
            {"max_cameras": 1},
            {"config_path": front_config},
        ],
        remappings=[
            ('/t265/fisheye1/image_raw', '/cam0/fisheye1/image_raw'),
            ('/t265/imu', '/cam0/imu'),
            # Isolate front's TF to avoid conflicts with EKF
            ('/tf', '/front/tf'),
            ('/tf_static', '/front/tf_static'),
        ]
    )

    # --- Back instance (cam1 + cam1 IMU) ---
    back_config = os.path.join(bringup_dir, 'config', 'estimator_config_back.yaml')
    back_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        name='ov_msckf',
        namespace='back',
        output='screen',
        parameters=[
            {"verbosity": "INFO"},
            {"use_stereo": False},
            {"max_cameras": 1},
            {"config_path": back_config},
        ],
        remappings=[
            ('/t265/fisheye1/image_raw', '/cam1/fisheye1/image_raw'),
            ('/t265/imu', '/cam1/imu'),
            # Isolate back's TF to avoid conflicts with front
            ('/tf', '/back/tf'),
            ('/tf_static', '/back/tf_static'),
        ]
    )

    return LaunchDescription([front_node, back_node])

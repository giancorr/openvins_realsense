import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('openvins_bringup')
    ov_config_path = os.path.join(bringup_dir, 'config', 'estimator_config.yaml')

    openvins_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        name='ov_msckf',
        namespace='front',
        output='screen',
        parameters=[
            {"verbosity": "INFO"},
            {"use_stereo": False},
            {"max_cameras": 1},
            {"config_path": ov_config_path},
        ],
        remappings=[
            ('/t265/fisheye1/image_raw', '/cam0/fisheye1/image_raw'),
            ('/t265/fisheye2/image_raw', '/cam1/fisheye1/image_raw'),
            ('/t265/imu', '/cam0/imu')
        ]
    )

    return LaunchDescription([openvins_node])

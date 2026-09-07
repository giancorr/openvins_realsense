import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('openvins_bringup')

    # --- Single Instance (Multi-Camera) ---
    config_path = os.path.join(bringup_dir, 'config', 'estimator_config_multicam.yaml')
    
    multicam_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        name='ov_msckf',
        output='screen',
        parameters=[
            {"verbosity": "INFO"},
            {"use_stereo": False},
            {"max_cameras": 2},
            {"config_path": config_path},
        ],
        remappings=[
            # We don't remap image topics here because kalibr_imucam_chain_multicam.yaml 
            # specifies /cam0/fisheye1/image_raw and /cam1/fisheye1/image_raw directly.
            # We also don't remap the IMU topic for the same reason.
        ]
    )

    return LaunchDescription([multicam_node])

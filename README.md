# OpenVINS RealSense Dual-VIO EKF Fusion

This repository provides a complete ROS2 (Humble) environment for tracking odometry using **two Intel RealSense T265 cameras** running independent instances of [OpenVINS](https://docs.openvins.com/), and fusing them into a single, perfectly smoothed trajectory using an Extended Kalman Filter (`robot_localization`).

## Architecture

1. **Dual OpenVINS Instances**: 
   - `cam0` (Front): Provides the absolute global pose (XYZ + Roll, Pitch, Yaw) acting as the spatial anchor.
   - `cam1` (Back): Provides redundant absolute position (XYZ) to improve stability and tracking robustness.
2. **Translation Cancellation Node (`odom_to_baselink.cpp`)**: 
   - Receives raw `nav_msgs/Odometry` from both VIOs.
   - Cancels the initial arbitrary translation offsets (the physical 22cm gap between the cameras) and the starting yaw, forcing both sensors to mathematically start exactly at `(0,0,0)` in the global frame.
   - Inflates the VIO measurement covariances by 100x to prevent the EKF from diverging due to sub-millimeter disagreements.
3. **EKF Fusion (`ekf.yaml`)**:
   - Uses `robot_localization` with a heavily tuned low-pass kinematic profile (`process_noise_covariance: 0.0001`).
   - Smoothly averages the two independent trajectories into a single, perfectly stable `base_link` output, completely eliminating high-frequency timestamp jitter and physical flexing vibrations.
4. **Dockerized Environment**:
   - Everything runs inside an isolated Docker container with GUI forwarding for `RViz2` and `PlotJuggler`.

## Running the System

1. Launch the Docker container:
   ```bash
   ./run.sh
   ```
2. Build the workspace (inside Docker):
   ```bash
   colcon build --packages-select odometry_tracker openvins_bringup
   ```
3. Run the Dual Session:
   ```bash
   tmux -u new-session -d -s dual_session "cd /root/ws; . install/setup.bash; ros2 launch openvins_bringup launch_openvins_dual.launch.py"
   tmux attach -t dual_session
   ```
   (Or use the provided `dual_session.yml` with tmuxinator).

## Frame Definitions
- `global`: The absolute origin `(0,0,0)` where the system initializes.
- `global_ned`: The North-East-Down orientation of the global frame.
- `base_link`: The exact center of the drone, tracking in FRD (Forward-Right-Down) convention.

## Hardware Setup
- Front Camera (`cam0`): +11 cm along the X-axis of `base_link`.
- Back Camera (`cam1`): -11 cm along the X-axis of `base_link`, physically mounted backwards.

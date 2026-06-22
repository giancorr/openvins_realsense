#!/bin/bash

# Allow local X11 connections for RViz
xhost +local:root > /dev/null 2>&1

echo "========================================="
echo "Starting OpenVINS + Realsense container..."
echo "NOTE: Make sure the camera is connected via USB!"
echo "========================================="
echo ""
echo "Once inside the container, run:"
echo "  1. cd /root/ros2_ws && colcon build --packages-select odometry_tracker openvins_bringup"
echo "  2. source install/setup.bash"
echo ""
echo "  SINGLE mode:  tmuxp load /run_config/session.yml"
echo "  DUAL mode:    tmuxp load /run_config/dual_session.yml"
echo ""

# Run the docker container
docker run -it --rm \
    --privileged \
    -v /dev:/dev \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e QT_X11_NO_MITSHM=1 \
    -v $(pwd):/run_config \
    --network host \
    openvins_realsense_humble bash -c "ln -sf /run_config/src/* /root/ros2_ws/src/ && exec bash"

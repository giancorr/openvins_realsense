FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# 1. Install common dependencies, OpenVINS deps, and RealSense build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    cmake \
    build-essential \
    curl \
    wget \
    nano \
    sudo \
    # OpenVINS specific dependencies
    libceres-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libsuitesparse-dev \
    libatlas-base-dev \
    libeigen3-dev \
    python3-dev \
    python3-matplotlib \
    python3-numpy \
    python3-psutil \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# 2. Clone the RealSense repository and OpenVINS fork
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/Simone-DAngelo/RealsenseT265.git && \
    git clone https://github.com/giancorr/open_vins.git

# 3. Build and install the low-level librealsense SDK 2.0 manually (T265)
WORKDIR /root/ros2_ws/src/RealsenseT265/librealsense/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j$(nproc) && \
    make install

# 4. Copy udev rules
RUN cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/ || true

# 5. Resolve ROS dependencies
WORKDIR /root/ros2_ws
RUN rosdep init || true
RUN rosdep update
RUN apt-get update && rosdep install -i --from-path src --rosdistro humble -y --skip-keys "librealsense2 catkin"

# 6. Prevent colcon from rebuilding the SDK redundantly
RUN touch /root/ros2_ws/src/RealsenseT265/librealsense/COLCON_IGNORE

# 7. Build the ROS 2 workspace (Realsense ROS wrapper + OpenVINS)
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -Drealsense2_DIR=/usr/local/lib/cmake/realsense2"

# 8. Install tmux, tmuxp, and robot_localization at the end to avoid invalidating the heavy build cache!
RUN apt-get update && apt-get install -y --no-install-recommends \
    tmux \
    tmuxp \
    ros-humble-robot-localization \
    ros-humble-plotjuggler-ros \
    && rm -rf /var/lib/apt/lists/*

# 9. Setup bashrc and entrypoint
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

RUN echo "#!/bin/bash\nset -e\nsource /opt/ros/humble/setup.bash\nsource /root/ros2_ws/install/setup.bash\nexec \"\$@\"" > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

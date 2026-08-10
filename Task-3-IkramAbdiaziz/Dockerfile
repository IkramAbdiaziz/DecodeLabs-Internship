FROM osrf/ros:humble-desktop

# Set non-interactive timezone
ENV DEBIAN_FRONTEND=noninteractive

# Update and install navigation system packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Set up container workspace
WORKDIR /workspace

# Copy local amr_navigation ROS 2 package
COPY amr_navigation ./src/amr_navigation

# Build the ROS 2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Add sourcing to root configuration profile
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Default launch command runs the integrated simulation launcher
ENTRYPOINT ["/bin/bash", "-c", "source /workspace/install/setup.bash && ros2 launch amr_navigation amr_navigation.launch.py"]

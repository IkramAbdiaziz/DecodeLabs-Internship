FROM osrf/ros:humble-desktop

# Install git, NLopt (C and C++ dev packages), kinematics, collision, and simulation dependencies
RUN apt-get update && apt-get install -y \
    git \
    libnlopt-dev \
    libnlopt-cxx-dev \
    ros-humble-moveit \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-xacro \
    xterm \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /workspace/src

# Clone official TRAC-IK source repository (since it is not available as a binary package in Humble apt)
RUN git clone https://github.com/traclabs/trac_ik.git

# Replace urdf/model.hpp with urdf/model.h for Humble compatibility in trac_ik_lib
RUN find /workspace/src/trac_ik -type f -exec sed -i 's|urdf/model.hpp|urdf/model.h|g' {} +

# Ignore compilation of unnecessary TRAC-IK plugin packages (which depend on newer MoveIt 2 headers)
RUN touch /workspace/src/trac_ik/trac_ik_kinematics_plugin/COLCON_IGNORE && \
    touch /workspace/src/trac_ik/trac_ik_examples/COLCON_IGNORE || true

# Copy our package source files
COPY . ./robotic_arm_kinematics

WORKDIR /workspace

# Source ROS 2 Humble and compile the entire workspace (colcon handles compilation order)
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Add setups to bashrc for interactive bash access
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Run the simulation launch script by default
CMD ["/bin/bash", "-c", "source install/setup.bash && ros2 launch robotic_arm_kinematics simulation.launch.py"]

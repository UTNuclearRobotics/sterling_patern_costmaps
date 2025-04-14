# Dockerfile
# Use ROS Noetic as the base image
FROM ros:noetic

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Calculate number of cores for parallel builds
RUN NUM_CORES=$(($(nproc) - 2)); \
    if [ "$NUM_CORES" -lt 1 ]; then NUM_CORES=1; fi; \
    echo "export NUM_CORES=${NUM_CORES}" >> ~/.bashrc

# Set environment variables
ENV ROS_DISTRO=noetic
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    ros-noetic-catkin \
    git \
    openssh-client \
    ros-noetic-nav-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-std-srvs \
    ros-noetic-tf \
    ros-noetic-move-base \
    ros-noetic-costmap-2d \
    ros-noetic-navfn \
    ros-noetic-dwa-local-planner \
    ros-noetic-slam-toolbox \
    ros-noetic-map-server \
    ros-noetic-robot-state-publisher \
    ros-noetic-rviz \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

#Install ROS Tools
RUN apt-get update && apt-get install -y \
    gazebo11 \
    ros-noetic-rviz \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-rqt \
    ros-noetic-slam-gmapping \
    ros-noetic-rqt-common-plugins \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstool \
    ros-noetic-dwa-local-planner \
    python3-pip

# Setup workspace
RUN mkdir -p /ros_ws/src
RUN mkdir -p /ros1_ws/src
WORKDIR /ros_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"


# Add GitHub's SSH host key to known_hosts
RUN mkdir -p ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Setup workspace
RUN mkdir -p /root/ros_ws/src
WORKDIR /root/ros_ws

# Clone your package (assuming sterling_patern_costmaps is part of utexas_sterling)
WORKDIR /root/ros_ws/src
RUN --mount=type=ssh git clone --branch noetic/working_dockerfile --depth=1 git@github.com:UTNuclearRobotics/sterling_patern_costmaps.git

# Build the workspace
WORKDIR /root/ros_ws
RUN source /opt/ros/noetic/setup.bash && catkin_make -j${NUM_CORES}

# Source the workspace in bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    && echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc \
    && source ~/.bashrc

# Create entrypoint script
RUN echo '#!/bin/bash' > /usr/bin/entrypoint.sh \
    && echo 'set -e' >> /usr/bin/entrypoint.sh \
    && echo '# Adjust UID and GID if provided' >> /usr/bin/entrypoint.sh \
    && echo 'if [ -n "$UID" ] && [ -n "$GID" ]; then' >> /usr/bin/entrypoint.sh \
    && echo '  groupmod -g $GID user || groupadd -g $GID user' >> /usr/bin/entrypoint.sh \
    && echo '  usermod -u $UID -g $GID user || useradd -m -u $UID -g $GID user' >> /usr/bin/entrypoint.sh \
    && echo '  export HOME=/home/user' >> /usr/bin/entrypoint.sh \
    && echo '  exec gosu user "$@"' >> /usr/bin/entrypoint.sh \
    && echo 'fi' >> /usr/bin/entrypoint.sh \
    && echo '# Source ROS environment' >> /usr/bin/entrypoint.sh \
    && echo 'source /opt/ros/noetic/setup.bash' >> /usr/bin/entrypoint.sh \
    && echo 'source /root/ros_ws/devel/setup.bash' >> /usr/bin/entrypoint.sh \
    && echo 'exec "$@"' >> /usr/bin/entrypoint.sh \
    && chmod +x /usr/bin/entrypoint.sh

# Install gosu for UID/GID switching
RUN apt-get update && apt-get install -y gosu \
    && rm -rf /var/lib/apt/lists/*

    RUN mkdir -p /root/dependency_ws/src
    WORKDIR /root/dependency_ws/src
    RUN --mount=type=ssh git clone https://github.com/ROBOTIS-GIT/turtlebot3.git \
        && cd turtlebot3 \
        && git checkout noetic
    RUN --mount=type=ssh git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git \
        && cd turtlebot3_simulations \
        && git checkout noetic
    WORKDIR /root/dependency_ws
    RUN rosdep update
    RUN rosdep install --from-paths src --ignore-src -r -y
    RUN source /opt/ros/noetic/setup.bash && catkin_make
    RUN echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc

# Set shell to login mode and default command
SHELL ["/bin/bash", "-l", "-c"]
CMD ["bash"]
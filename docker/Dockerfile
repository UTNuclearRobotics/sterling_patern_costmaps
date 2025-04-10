# Dockerfile
FROM ros:noetic

SHELL ["/bin/bash", "-c"]

RUN NUM_CORES=$(($(nproc) - 2)); \
    if [ "$NUM_CORES" -lt 1 ]; then NUM_CORES=1; fi;

ENV ROS_DISTRO noetic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies (example: build-essential and ros tools)
RUN apt-get update && apt-get install -y \
    build-essential \
    ros-noetic-catkin \
    git \
    openssh-client \
    ros-noetic-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

#Install ROS Tools
RUN apt-get update && apt-get install -y \
    gazebo11 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-rqt \
    ros-noetic-rqt-common-plugins \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstool \
    python3-pip

# Setup workspace
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"


# Add GitHub's SSH host key to known_hosts
RUN mkdir -p ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Clone Turtlebot3 packages
WORKDIR /root/dependency_ws/src

#Turtlebot 3 Dependencies
RUN --mount=type=ssh git clone https://github.com/ROBOTIS-GIT/turtlebot3.git \
    && cd turtlebot3 \
    && git checkout noetic

RUN --mount=type=ssh git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git \
    && cd turtlebot3_simulations \
    && git checkout noetic

#From root, go to depend ws and install all the dependenies
WORKDIR /root/dependency_ws
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make
ENV TURTLEBOT3_MODEL=waffle
# Source ROS
CMD ["bash"]
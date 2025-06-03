# Use ROS Noetic as the base image
FROM ros:noetic-robot

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Calculate number of cores for parallel builds and set as environment variable
ENV NUM_CORES=8
RUN NUM_CORES=$(($(nproc) - 2)); \
    if [ "$NUM_CORES" -lt 1 ]; then NUM_CORES=1; fi; \
    echo "export NUM_CORES=${NUM_CORES}" >> ~/.bashrc

# Set environment variables
ENV ROS_DISTRO=noetic
ENV DEBIAN_FRONTEND=noninteractive

RUN rm /etc/apt/sources.list.d/ros1-latest.list && \
    apt update && apt install curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros1.list > /dev/null

RUN apt update && sudo apt install ros2-apt-source ; rm /etc/apt/sources.list.d/ros1.list; rm /usr/share/keyrings/ros-archive-keyring.gpg

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt install curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    openssh-client \
    ros-noetic-catkin \
    ros-noetic-amcl \
    ros-noetic-xacro \
    ros-noetic-joint-state-publisher \
    ros-noetic-rosserial-python \
    ros-noetic-hls-lfcd-lds-driver \
    ros-noetic-turtlebot3-msgs \
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
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# Install ROS Tools
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

RUN pip install \
    inflection \
    rdflib \
    wrapt \
    termcolor \
    tqdm \
    opencv-python \
    numpy \
    joblib \
    scikit-learn \
    torch \
    pyyaml \
    tensorflow \
    torchvision

# Add GitHub's SSH host key to known_hosts
RUN mkdir -p ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Setup workspace
RUN mkdir -p /root/ros_ws/src
WORKDIR /root/ros_ws

# Clone your package into src/
WORKDIR /root/ros_ws/src
RUN --mount=type=ssh git clone --branch noetic/working_dockerfile --depth=1 git@github.com:UTNuclearRobotics/sterling_patern_costmaps.git
RUN rm -rf /turtlebot3 
RUN --mount=type=ssh git clone --branch noetic-feature/camera-change --depth=1 git@github.com:UTNuclearRobotics/turtlebot3.git

# # Create entrypoint script
# RUN echo '#!/bin/bash' > /usr/bin/entrypoint.sh \
#     && echo 'set -e' >> /usr/bin/entrypoint.sh \
#     && echo '# Adjust UID and GID if provided' >> /usr/bin/entrypoint.sh \
#     && echo 'if [ -n "$UID" ] && [ -n "$GID" ]; then' >> /usr/bin/entrypoint.sh \
#     && echo '  groupmod -g $GID user || groupadd -g $GID user' >> /usr/bin/entrypoint.sh \
#     && echo '  usermod -u $UID -g $GID user || useradd -m -u $UID -g $GID user' >> /usr/bin/entrypoint.sh \
#     && echo '  export HOME=/home/user' >> /usr/bin/entrypoint.sh \
#     && echo '  exec gosu user "$@"' >> /usr/bin/entrypoint.sh \
#     && echo 'fi' >> /usr/bin/entrypoint.sh \
#     && echo '# Source ROS environment' >> /usr/bin/entrypoint.sh \
#     && echo 'source /opt/ros/noetic/setup.bash' >> /usr/bin/entrypoint.sh \
#     && echo 'source /root/ros_ws/devel/setup.bash' >> /usr/bin/entrypoint.sh \
#     && echo 'exec "$@"' >> /usr/bin/entrypoint.sh \
#     && chmod +x /usr/bin/entrypoint.sh

# # Install gosu for UID/GID switching
# RUN apt-get update && apt-get install -y gosu \
#     && rm -rf /var/lib/apt/lists/*

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
RUN source /opt/ros/noetic/setup.bash && catkin build
RUN echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source /root/dependency_ws/devel/setup.bash" >> ~/.bashrc \
    && source ~/.bashrc

RUN echo "source /root/ros_ws/src/sterling_patern_costmaps/alias.sh" >> ~/.bashrc

# copy the entrypoint into the image
COPY docker/entrypoint.sh /usr/bin/entrypoint.sh
RUN chmod +x /usr/bin/entrypoint.sh

# run this script on startup
ENTRYPOINT /usr/bin/entrypoint.sh

WORKDIR /root/ros_ws

# Set shell to login mode and default command
SHELL ["/bin/bash", "-l", "-c"]
CMD ["bash"]
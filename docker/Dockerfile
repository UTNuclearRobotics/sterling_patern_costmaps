FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

# set the ros version
ENV ROS_DISTRO humble
ENV DEBIAN_FRONTEND=noninteractive
ENV NUM_CORES=22

# install some useful packages and upgrade existing ones
RUN apt update && apt upgrade -V -y \
    && apt install -y \
    apt-utils \
    git \
    curl \
    nano \
    vim \
    tmux \
    x11-apps \
    python3-pip \
    python3-colcon-clean \
    openssh-client \
    lsb-release \
    gnupg \
    ignition-fortress

RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash \
    && apt install -y git-lfs \
    && git-lfs install \
    && apt install -y libcurl4-openssl-dev

# install ros packages
RUN apt install -y \
    ros-${ROS_DISTRO}-rqt-graph \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-opennav-docking \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-ign-ros2-control

# Sterling dependencies
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
    
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Add GitHub's SSH host key to known_hosts
RUN mkdir -p ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Clone Husarion Panther packages
WORKDIR /root/dependency_ws/src

RUN --mount=type=ssh git clone -b humble https://github.com/BehaviorTree/BehaviorTree.ROS2.git
RUN --mount=type=ssh git clone -b ros2 https://github.com/husarion/panther_msgs.git
RUN --mount=type=ssh git clone -b ros2-devel https://github.com/husarion/wibotic_ros.git
RUN --mount=type=ssh git clone https://github.com/husarion/husarion_gz_worlds.git

# DepthAI ROS
RUN --mount=type=ssh git clone git@github.com:UTNuclearRobotics/depthai-ros.git \
    && cd depthai-ros \
    && git checkout e104466be117b9094553001b73419ebcb4cca1ec
# husarion_ugv_autonomy_ros
RUN --mount=type=ssh git clone https://github.com/husarion/husarion_ugv_autonomy_ros.git \
    && cd husarion_ugv_autonomy_ros \
    && git checkout fdcd75701673212962703c600ab12c23ddecd6ba
# ROS Components Description
RUN --mount=type=ssh git clone git@github.com:UTNuclearRobotics/ros_components_description.git \
    && cd ros_components_description \
    && git checkout b33b2655241e1aa123fd8c2de0b5b7d0310e36f2
# husarion_ugv_ros
RUN --mount=type=ssh git clone git@github.com:UTNuclearRobotics/husarion_ugv_ros.git \
    && cd husarion_ugv_ros \
    && git checkout b1aa62f7160b9a3765ef360eae9a5779e73bda95
# Sterling Gazebo
RUN --mount=type=ssh git clone git@github.com:UTNuclearRobotics/sterling_gazebo.git \
    && cd sterling_gazebo \
    && git checkout 0fe39aa8483e650973286b760a8d6288d50854f5
# Panther accessories including nav2 bringup and params
RUN --mount=type=ssh git clone git@github.com:UTNuclearRobotics/utexas_panther_accessories.git \
    && cd utexas_panther_accessories \
    && git checkout 424e22e076f77b68cc5eeb0fd885f81dc7fab959

# Install deps
WORKDIR /root/dependency_ws
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y
# TODO: the --continue-on-error should not be needed.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build

# Remove unnecessary files
RUN rm -rf src build logs

# setup .bashrc
SHELL ["/bin/bash", "-l", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source /root/dependency_ws/install/setup.bash" >> ~/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.profile \
    && echo "source /root/dependency_ws/install/setup.bash" >> ~/.profile \
    && source ~/.bashrc

COPY alias.sh /root/alias.sh
RUN echo "source /root/alias.sh" >> ~/.bashrc

# copy the entrypoint into the image
COPY docker/entrypoint.sh /usr/bin/entrypoint.sh
RUN chmod +x /usr/bin/entrypoint.sh

WORKDIR /root/ros2_ws/

# run this script on startup
ENTRYPOINT /usr/bin/entrypoint.sh
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

# set the ros version
ENV ROS_DISTRO humble
ENV DEBIAN_FRONTEND=noninteractive

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
    python3-pip

RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash \
    && apt install -y git-lfs \
    && git-lfs install \
    && apt install -y libcurl4-openssl-dev

# install ros packages
# see ../docs/installing_ros_packages.md for alternatives
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
    ros-${ROS_DISTRO}-nav2-bringup

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

RUN sudo apt-get update
RUN sudo apt-get install lsb-release gnupg
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Fortress
RUN apt update && apt install -y \
    ignition-fortress \
    ros-${ROS_DISTRO}-ign-ros2-control

# Unsecurely put private SSH key in the image to clone private repos
RUN apt update && apt install -y git openssh-client
COPY id_rsa /root/.ssh/id_rsa
RUN chmod 600 /root/.ssh/id_rsa

# Add GitHub's SSH host key to known_hosts
RUN mkdir -p /root/.ssh && \
    touch /root/.ssh/known_hosts && \
    chmod 600 /root/.ssh/known_hosts && \
    ssh-keyscan github.com >> /root/.ssh/known_hosts

# Clone Husarion Panther packages
WORKDIR /root/dependency_ws/src

RUN git clone -b humble https://github.com/BehaviorTree/BehaviorTree.ROS2.git
RUN git clone -b ros2 https://github.com/husarion/panther_msgs.git
RUN git clone https://github.com/husarion/husarion_gz_worlds.git

# DepthAI ROS
RUN git clone git@github.com:UTNuclearRobotics/depthai-ros.git \
    && cd depthai-ros \
    && git checkout e104466be117b9094553001b73419ebcb4cca1ec
# Panther ROS
RUN git clone git@github.com:UTNuclearRobotics/panther_ros.git \
    && cd panther_ros \
    && git checkout 126e91221922e7f738e63ce73fb6f08345cf2f58
# Sterling Gazebo
RUN git clone git@github.com:UTNuclearRobotics/sterling_gazebo.git \
    && cd sterling_gazebo \
    && git checkout ec7141214fef1bbd32800086ce4c3fc1ea8bb330
# NAV2 Docker
RUN git clone git@github.com:UTNuclearRobotics/nav2-docker.git \
    && cd nav2-docker \
    && git checkout c9e5ca43598b208dba7a88aa928407ae6cf67218
# ROS Components Description
RUN git clone git@github.com:UTNuclearRobotics/ros_components_description.git \
    && cd ros_components_description \
    && git checkout a31f6f6fe1f5f3530a79b219f967911e2cef6a78

# Remove the private SSH key after cloning the repositories
RUN rm /root/.ssh/id_rsa

# Install deps
WORKDIR /root/dependency_ws
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y
# TODO: the --continue-on-error should not be needed.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --continue-on-error

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

# Commands to run inside the container

### Step 1: Choose the terrain resolution you want ot run the Gazebo world
alias run_gazebo_low='ros2 launch sterling_gazebo sidewalks.launch.py res:=low'
alias run_gazebo_medium='ros2 launch sterling_gazebo sidewalks.launch.py res:=medium'
alias run_gazebo_high='ros2 launch sterling_gazebo sidewalks.launch.py res:=high'

### Step 2: Launch Nav2
# Sterling parameters
alias run_nav2='ros2 launch husarion_nav2 navigation2_bringup.launch.py use_rviz:=True use_sim_time:=True nav2_config_file_slam:=/root/ros2_ws/src/sterling/config/nav2_params.yaml'

# Run Nav2 with a loaded map. The first argument is the map file path and should contain a .pgm and .yaml file.
# function run_nav2_loaded() {
#     if [ ! -f "$1" ]; then
#         echo "Error: The provided argument is not a valid file path."
#         return 1
#     fi

#     ros2 launch husarion_nav2 navigation2_bringup.launch.py \
#     use_rviz:=True \
#     use_sim_time:=True \
#     nav2_config_file_slam:=/root/ros2_ws/src/sterling/config/nav2_params.yaml \
#     map:="$1"; #TODO: Map paremeter
# };

# Default parameters, when want to collect rosbag data
alias run_nav2_default='ros2 launch husarion_nav2 navigation2_bringup.launch.py use_rviz:=True use_sim_time:=True'

### Step 3: Sterling costmap node
alias run_sterling_costmap='source /root/ros2_ws/install/setup.bash && ros2 launch sterling costmaps.launch.py'
# While node is running, call service to save the costmap
alias save_costmap='source /root/ros2_ws/install/setup.bash && ros2 service call /sterling/save_costmap std_srvs/srv/Trigger'
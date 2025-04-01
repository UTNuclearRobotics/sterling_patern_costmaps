# Commands to run inside the container

### Step 1: Choose the terrain resolution you want ot run the Gazebo world
alias run_gazebo_low='ros2 launch sterling_gazebo sidewalks.launch.py res:=low'
alias run_gazebo_medium='ros2 launch sterling_gazebo sidewalks.launch.py res:=medium'
alias run_gazebo_high='ros2 launch sterling_gazebo sidewalks.launch.py res:=high'

### Step 2: Launch Nav2
# Sterling parameters
alias run_nav2='ros2 launch utexas_panther bringup_launch.py namespace:=panther observation_topic:=ouster/scan observation_topic_type:=laserscan slam:=true use_rviz:=True use_sim_time:=true params_file:=/root/ros2_ws/src/sterling/config/nav2_params.yaml'

# How to send a goal_pose now and the alias does not work
function run_goal() {
    local x=${1:-0.0}
    local y=${2:-0.0}
    ros2 topic pub /panther/goal_pose geometry_msgs/msg/PoseStamped "{ header: { frame_id: 'panther/map' }, pose: { position: { x: $x, y: $y }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }"
}

### Step 3: Sterling costmap node
alias run_sterling_costmap='source /root/ros2_ws/install/setup.bash && ros2 launch sterling costmaps.launch.py'
# While node is running, call service to save the costmap
alias save_costmap='source /root/ros2_ws/install/setup.bash && ros2 service call /sterling/save_costmap std_srvs/srv/Trigger'
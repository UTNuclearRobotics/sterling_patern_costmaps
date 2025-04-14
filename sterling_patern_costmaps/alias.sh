# Commands to run inside the container

### Step 1: Choose the terrain resolution you want ot run the Gazebo world
alias run_gazebo_low='roslaunch sterling_gazebo sidewalks.launch.py high_res:=false namespace:=panther'
alias run_gazebo_high='roslaunch sterling_gazebo sidewalks.launch.py high_res:=true namespace:=panther'

### Step 2: Launch Nav2
# Sterling parameters
alias run_nav='roslaunch sterling_patern_costmaps costmaps.launch'

# How to send a goal_pose now and the alias does not work
function run_goal() {
    local x=${1:-0.0}
    local y=${2:-0.0}
    rostopic pub /panther/sterling/move_base_simple/goal geometry_msgs/PoseStamped "header: { frame_id: 'panther/sterling/map' }, pose: { position: { x: $x, y: $y, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } }" -1
}

### Step 3: Sterling costmap node
alias run_sterling_patern_costmap='source /root/ros_ws/devel/setup.bash && roslaunch sterling_patern_costmaps costmaps.launch'
# While node is running, call service to save the costmap
alias save_costmap='source /root/ros_ws/devel/setup.bash && rosservice call /sterling_patern_costmaps/save_costmap "{}"'
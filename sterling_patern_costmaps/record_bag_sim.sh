source /opt/ros/humble/setup.bash

BAG_NAME="panther_sim_recording_$(date +%Y%m%d_%H%M%S)"
BAG_PATH="$(pwd)/$BAG_NAME"
echo "Recording bag file..."
echo "Outputting bag to ${BAG_PATH}"

ros2 bag record -o ${BAG_PATH} \
    /oakd2/oak_d_node/rgb/camera_info \
    /oakd2/oak_d_node/rgb/image_rect_color \
    /imu/data \
    /odometry/filtered
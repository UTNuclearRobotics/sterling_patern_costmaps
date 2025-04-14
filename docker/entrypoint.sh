#!/bin/bash

set -e

# Adjust UID and GID if provided
if [ -n "$UID" ] && [ -n "$GID" ]; then
  # Check if 'user' group exists and modify it, or create a new group
  groupmod -g "$GID" user 2>/dev/null || groupadd -g "$GID" user
  # Check if 'user' exists and modify it, or create a new user
  usermod -u "$UID" -g "$GID" user 2>/dev/null || useradd -m -u "$UID" -g "$GID" user
  export HOME=/home/user
  # Execute the command as the specified user
  exec gosu user "$@"
fi

# Source ROS1 Noetic environment
source /opt/ros/noetic/setup.bash

# Source the workspace
source /root/ros_ws/devel/setup.bash

# Execute the command passed to the container
exec "$@"
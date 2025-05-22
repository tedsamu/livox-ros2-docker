#!/bin/bash

# Change MID360 sensor IP address if provided
if [[ $1 =~ ^[0-9]+$ ]]; then
    sed -i "s/\"ip\"[ ]*:[ ]*\"192.168.1.[0-9]\+\"/\"ip\" : \"192.168.1.1$1\"/" /home/devuser/livox_ws/src/livox_ros_driver2/config/MID360_config.json
    shift
fi

# Source the necessary ROS2 and workspaces
source /opt/ros/humble/setup.bash
source /home/devuser/livox_ws/install/setup.bash
# This will fail in the base livox container since it doesn't use that workspace. Hence I send the error to /dev/null.
source /home/devuser/ros2_ws/install/setup.bash 2> /dev/null
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
# Run the provided command
$@

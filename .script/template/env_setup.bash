#!/bin/bash

: "${DOGBOT_PATH:=/workspaces/dogbot_runtime}"

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RCUTILS_COLORIZED_OUTPUT=1

source /opt/ros/humble/setup.bash

if [ -f "/dogbot_install/local_setup.bash" ]; then
    source /dogbot_install/local_setup.bash
elif [ -f "${DOGBOT_PATH}/ros2_ws/install/local_setup.bash" ]; then
    source "${DOGBOT_PATH}/ros2_ws/install/local_setup.bash"
fi

export PATH="${PATH}:${DOGBOT_PATH}/.script"

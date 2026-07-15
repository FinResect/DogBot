#!/bin/zsh

: "${DOGBOT_PATH:=/workspaces/DogBot}"

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RCUTILS_COLORIZED_OUTPUT=1

source /opt/ros/humble/setup.zsh

if [ -f "/dogbot_install/local_setup.zsh" ]; then
    source /dogbot_install/local_setup.zsh
elif [ -f "${DOGBOT_PATH}/ros2_ws/install/local_setup.zsh" ]; then
    source "${DOGBOT_PATH}/ros2_ws/install/local_setup.zsh"
fi

eval "$(register-python-argcomplete ros2)"
eval "$(register-python-argcomplete colcon)"

export PATH="${PATH}:${DOGBOT_PATH}/.script"

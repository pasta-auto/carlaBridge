#!/bin/bash -e

export CARLA_ROOT=/root/carlaPython
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/util
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/carla
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/carla/agents
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/carla/dist/carla-0.9.14-py3.10-linux-x86_64.egg

export SCRIPT_ROOT="/root/carlaBridge"

DEFAULT_WSL_HOST=$(ip route show default | awk '/default/ {print $3}')

export AGENT_FRAME_RATE="20"
# Autonomous actor default role_name
export AGENT_ROLE_NAME="heroAW"

# export Carla IP Location
# The default is the WSL2 host IP address
# Please change accordingly
export SIMULATOR_LOCAL_HOST="$DEFAULT_WSL_HOST"
export SIMULATOR_PORT="2000"

# Enable mode2 by defining the connection port
# If not defined mode2 is not enabled
export SIMULATOR_MODE2_PORT="3000"

# Setup requirements for ROS2
export ROS_LOCALHOST_ONLY=1
ip link set lo multicast on
sysctl -w net.core.rmem_max=2147483647

# Prep any files needed and do exports of envs from Carla 
varsToExport=$(python3 ${SCRIPT_ROOT}/carla_bridge_prep.py)
eval ${varsToExport}

#gnome-terminal -- bash -c roscore
ros2 daemon start

# /pasta-interface/server/startServers.sh
/root/carlaBridge/start_ros2.sh ${AGENT_ROLE_NAME} ${CARLA_TOWN_MAP} true ${DEFAULT_WSL_HOST} 2000

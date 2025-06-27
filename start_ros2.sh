#! /bin/bash -e
role_name=$1
map_name=$2
explore_mode=$3
carla_ip=$4
carla_port=$5
route_topic=$6

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

echo "-------------------------------"
echo "Starting OpenPlanner .. "
echo $role_name
echo $map_name
echo $explore_mode
echo $carla_ip:$carla_port
echo $route_topic
echo "-------------------------------"

source /autoware/install/setup.bash

cp -r ${SCRIPT_DIR}/autoware_universe_modifications/* ${COLCON_PREFIX_PATH}/

if [ ! -d "/root/maps/$map_name" ]
then
    echo -e "\n\n$map_name does not exist in the maps folder, exiting!!!\n\n"
    exit 1
fi

ros2 launch ${COLCON_PREFIX_PATH}/autoware_launch/share/autoware_launch/launch/autoware.launch.xml map_path:=/root/maps/${map_name} vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit carla_role_name:=${role_name} carla_ip:=${carla_ip} carla_port:=${carla_port}
#$SHELL

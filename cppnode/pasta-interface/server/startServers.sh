#!/bin/bash

#SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
ARG1=${1:-apiServers/}
${ARG1}can_server &
${ARG1}camera_server & 
${ARG1}gnss_server   & 
${ARG1}imu_server    & 
${ARG1}lidar_server  & 
${ARG1}radar_server  & 
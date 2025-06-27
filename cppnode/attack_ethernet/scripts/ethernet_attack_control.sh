#!/bin/bash

IFS=","

showHelp() {
cat << EOF  
Usage: ./ethernet_attack_control.sh <options>
Control the Ethernet attack system for Carla - Autoware integration

Valid targets:
    all
    camera
    camera_info
    lidar
    nav_sat
    ecef_vel
    imu
    gnss_ins
    steering_status
    hazard_status
    turning_status
    gear_report
    velocity_report
    actuation_status
    handbrake_status
    headlights_status
    twist

--target        Comma-separated list of targets (or all)

--replaypercent Percentage of messages to replay (0.0 - 1.0)
--replaycount   Number of times each message should be replayed
--replaydelay   Amount of time (seconds) between each replay

--dospercent    Drop percentage for each message (0.0 - 1.0)
--dosdelay      Amount of time (seconds) to delay each incoming message
--doscount      Number of times to duplicate each message

-s|--status     Prints the current status of the settings and exits

--reset         Resets the config settings to defaults and disables the attack

EOF
# EOF 
}

printSettings() {
    status=$(ros2 topic echo /attack/ethernet/status --once --field config --no-daemon)

    echo "$status"
}

options=$(getopt -l "status,reset,target:,replaycount:,replaydelay:,replaypercent:,dosdelay:,dospercent:,doscount:" -o "s" -aq -- "$@")

if [ $? -ne 0 ]; then
    showHelp
    exit 0
fi

eval set -- "$options"

# Checks for number inputs
re='^[+-]?[0-9]+([.][0-9]+)?$'

reInt='^[0-9]+$'

attack_type=0
target=0

while true
do
case "$1" in
--target)
    shift
    for t in $1
    do
        case "$t" in
        all)
            target=$(($target|0x3FFF))
            break
        ;;
        camera)
            target=$(($target|0x1))
        ;;
        camera_info)
            target=$(($target|0x2))
        ;;
        lidar)
            target=$(($target|0x4))
        ;;
        nav_sat)
            target=$(($target|0x8))
        ;;
        ecef_vel)
            target=$(($target|0x10))
        ;;
        imu)
            target=$(($target|0x20))
        ;;
        gnss_ins)
            target=$(($target|0x40))
        ;;
        steering_status)
            target=$(($target|0x80))
        ;;
        hazard_status)
            target=$(($target|0x100))
        ;;
        turning_status)
            target=$(($target|0x200))
        ;;
        gear_report)
            target=$(($target|0x400))
        ;;
        velocity_report)
            target=$(($target|0x800))
        ;;
        actuation_status)
            target=$(($target|0x1000))
        ;;
        handbrake_status)
            target=$(($target|0x2000))
        ;;
        headlights_status)
            target=$(($target|0x4000))
        ;;
        twist)
            target=$(($target|0x8000))
        ;;
        *)
            echo -e "Invalid target: $t\n"
            showHelp
            exit -1
            break;;
        esac
    done
    ;;
--replaycount)
    shift
    if [[ $1 =~ $re ]]; then
        replaycount=$1
        attack_type=$(($attack_type|0x1))
    else
        echo -e "Invalid replay count entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--replaydelay)
    shift
    if [[ $1 =~ $re ]]; then
        replaydelay=$1
        attack_type=$(($attack_type|0x1))
    else
        echo -e "Invalid replay delay entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--replaypercent)
    shift
    if [[ $1 =~ $re ]]; then
        replaypercent=$1
        attack_type=$(($attack_type|0x1))
    else
        echo -e "Invalid replay percent entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--dosdelay)
    shift
    if [[ $1 =~ $re ]]; then
        dosdelay=$1
        attack_type=$(($attack_type|0x2))
    else
        echo -e "Invalid dos delay entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--dospercent)
    shift
    if [[ $1 =~ $re ]]; then
        dospercent=$1
        attack_type=$(($attack_type|0x2))
    else
        echo -e "Invalid dos percent entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--doscount)
    shift
    if [[ $1 =~ $re ]]; then
        doscount=$1
        attack_type=$(($attack_type|0x2))
    else
        echo -e "Invalid dos count entry $1\n"
        showHelp
        exit -1
    fi
    ;;
-s|--status)
    printSettings
    exit 0
    ;;
--reset)
    echo "Resetting Ethernet Attack Config"
    ros2 topic pub --once /attack/ethernet/config attack_ethernet_msgs/msg/EthernetAttackConfig "attack_type: 0
attack_target: 0
replay_count: 0
replay_delay: 0
replay_percent: 0
dos_delay: 0
dos_drop_percent: 0
dos_count: 0"
    exit 0
    ;;
--)
    shift
    break;;
*)
    showHelp
    exit 0
    break;;
esac
shift
done

command_string=$'ros2 topic pub --once /attack/ethernet/config attack_ethernet_msgs/msg/EthernetAttackConfig "'

if [ $attack_type -eq 0 ]; then
    echo -e "No options were selected\n"
    showHelp
    exit 0
fi

command_string+=$'\nattack_type: '$attack_type
command_string+=$'\nattack_target: '$target    

if (($attack_type&0x1)); then
    if [ ! -z "$replaycount" ]; then
        command_string+=$'\nreplay_count: '$replaycount    
    fi
    if [ ! -z "$replaydelay" ]; then
        command_string+=$'\nreplay_delay: '$replaydelay    
    fi
    if [ ! -z "$replaypercent" ]; then
        command_string+=$'\nreplay_percent: '$replaypercent    
    fi
fi

if (($attack_type&0x2)); then
    if [ ! -z "$dosdelay" ]; then
        command_string+=$'\ndos_delay: '$dosdelay    
    fi
    if [ ! -z "$dospercent" ]; then
        command_string+=$'\ndos_drop_percent: '$dospercent    
    fi
    if [ ! -z "$doscount" ]; then
        command_string+=$'\ndos_count: '$doscount    
    fi
fi
command_string+=$'"'

eval $command_string

printSettings


#!/bin/bash

IFS=","
PI="3.141592653589793"

showHelp() {
# `cat << EOF` This means that cat should stop reading when EOF is detected
cat << EOF  
Usage: ./lidar_attack_control.sh <options>
Control the Lidar attack system for Carla - Autoware integration

--attack         This is a override for setting the attack type regardless of other options.
--angle          Sets the attack angle filter (degrees)
--angle-drop     Angle dropoff                (degrees)
--dless          Distance less then attack filter    (meters)
--dgreater       Distance greater then attack filter (meters)
--distance-drop  Distance dropoff in meters          (meters)
--zmin           Z height minimum attack filter, anything over this height is removed (meters)
--ztol           Z height attack filter tolerance due to floating points              (meters)
--lanelet        Lanelet ID to use for attack filter
--box            The bound box coordinates x1,y1,z1,x2,y2,z2
--rpercent       Random percentage to remove on attack that matches filters (0.0 to 1.0)
-s|--status      Prints the current status of the settings and exits
--reset          Resets the config settings to defaults and disables the attack

EOF
# EOF 
}

printSettings() {
    status=$(ros2 topic echo /attack/lidar_blind/status --once --field config --no-daemon)

    echo $status
}

options=$(getopt -l "status,reset,attack:,angle:,angle-drop:,dless:,dgreater:,distance-drop:,zmin:,ztol:,rpercent:,lanelet:,x1:,x2:,y1:,y2:,z1:,z2,box:" -o "s" -aq -- "$@")

if [ $? -ne 0 ]; then
    showHelp
    exit 0
fi

eval set -- "$options"

# Checks for number inputs
re='^[+-]?[0-9]+([.][0-9]+)?$'
reBox='^([+-]?[0-9]+([.][0-9]+)?,?){6}'

reInt='^[0-9]+$'

non_attack_type=false
attack_type=0
dless=0
dgreater=0

while true
do
case "$1" in
--attack) 
    shift
    if [[ $1 =~ $re ]]; then
        attack_type_override=$1
    else
        echo -e "Invalid attack_type entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--angle) 
    shift
    if [[ $1 =~ $re ]]; then
        angle=$1
        angle=$(echo "$angle*$PI/180" | bc -l)
        attack_type=$(($attack_type|0x1))
    else
        echo -e "Invalid angle entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--angle-drop) 
    shift
    if [[ $1 =~ $re ]]; then
        angle_drop="$1"
        angle_drop=$(echo "$angle_drop*$PI/180" | bc -l)
        non_attack_type=true
    else
        echo -e "Invalid angle drop entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--dless)
    shift
    if [[ $1 =~ $re ]]; then
        dless=$1
        attack_type=$(($attack_type|0x2))
    else
        echo -e "Invalid dless entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--dgreater)
    shift
    if [[ $1 =~ $re ]]; then
        dgreater=$1
        attack_type=$(($attack_type|0x2))
    else
        echo -e "Invalid dgreater entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--distance-drop)
    shift
    if [[ $1 =~ $re ]]; then
        distance_drop=$1
        non_attack_type=true
    else
        echo -e "Invalid distance drop entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--zmin)
    shift
    if [[ $1 =~ $re ]]; then
        zmin=$1
        attack_type=$(($attack_type|0x4))
    else
        echo -e "Invalid zmin entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--ztol)
    shift
    if [[ $1 =~ $re ]]; then
        ztol=$1
        attack_type=$(($attack_type|0x4))
    else
        echo -e "Invalid ztol entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--rpercent)
    shift
    if [[ $1 =~ $re ]]; then
        rpercent=$1
        attack_type=$(($attack_type|0x8))
    else
        echo -e "Invalid rpercent entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--box)
    shift
    if [[ $1 =~ $reBox ]]; then
        bound_box=$1
        attack_type=$(($attack_type|0x20))
    else
        echo -e "Invalid box entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--lanelet)
    shift
    if [[ $1 =~ $reInt ]]; then
        lanelet=$1
        attack_type=$(($attack_type|0x10))
    else
        echo -e "Invalid lanelet entry $1\n"
        showHelp
        exit -1
    fi
    ;;
-s|--status)
    printSettings
    exit 0
    ;;
--reset)
    echo "Resetting Lidar Attack Config"
    ros2 topic pub --once /attack/lidar_blind/config attack_lidar_blind_msgs/msg/LidarAttackConfig "attack_type: 0
attack_angle: 0.0
attack_angle_dropoff: 0.0349066
distance_dropoff: 0.0
distance_less: 0.0
distance_greater: 0.0
z_min: 0.1
z_tolerance: 0.01
random_drop_percent: 0.95
lanelet_id: 0
bound_box:
- x: 0.0
  y: 0.0
  z: 0.0
- x: 0.0
  y: 0.0
  z: 0.0"
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

command_string=$'ros2 topic pub --once /attack/lidar_blind/config attack_lidar_blind_msgs/msg/LidarAttackConfig "'

# echo $(($attack_type&0x1))
if [ ! -z $attack_type_override ]; then
    attack_type=$attack_type_override
fi

if [ $attack_type -eq 0 ] && [ -z $attack_type_override ] && [ ! $non_attack_type ]; then
    echo -e "No options were selected\n"
    showHelp
    exit 0
fi

if [ ! -z "$bound_box" ]; then
    read bound_x1 bound_y1 bound_z1 bound_x2 bound_y2 bound_z2 <<< "$bound_box"
    command_string+=$'\nbound_box:\n- x: '$bound_x1$'\n  y: '$bound_y1$'\n  z: '$bound_z1
    command_string+=$'\n- x: '$bound_x2$'\n  y: '$bound_y2$'\n  z: '$bound_z2
fi

if [ ! -z "$angle_drop" ]; then
    command_string+=$'\nattack_angle_dropoff: '$angle_drop    
fi

if [ ! -z "$distance_drop" ]; then
    command_string+=$'\ndistance_dropoff: '$distance_drop    
fi

command_string+=$'\nattack_type: '$attack_type

if (($attack_type&0x1)); then
    command_string+=$'\nattack_angle: '$angle    
fi

if (($attack_type&0x2)); then
    if ((dless <= dgreater)); then
        echo "$dgreater > distance < $dless"
        echo "dless(distance less than)=$dless is lower then dgreater(distance greater then)=$dgreater"
        echo "dless defines the upper bounds of the distance checking."
        echo "dgreater defines the lower bounds of the distance checking."
        echo "please make dless higher then dgreater"
        exit 1
    fi

    command_string+=$'\ndistance_less: '$dless
    command_string+=$'\ndistance_greater: '$dgreater
fi

if (($attack_type&0x4)); then
    if [ ! -z $zmin ]; then
        command_string+=$'\nz_min: '$zmin
    fi

    if [ ! -z $ztol ]; then
        command_string+=$'\nz_tolerance: '$ztol
    fi
fi

if (($attack_type&0x8)); then
    command_string+=$'\nrandom_drop_percent: '$rpercent
fi

if (($attack_type&0x10)); then
    command_string+=$'\nlanelet_id: '$lanelet
fi

command_string+=$'"'

eval $command_string

if (($attack_type&0x10)); then
    status_lanelet=$(ros2 topic echo /attack/lidar_blind/status --once --field config.attack_type -s --csv) 
    
    if [ $status_lanelet -ne $attack_type ]; then
        echo -e "lanelet_id $laneled_id does not appear to be valid. The attack type has been set to 0(disabled)\n"
    fi
fi

printSettings


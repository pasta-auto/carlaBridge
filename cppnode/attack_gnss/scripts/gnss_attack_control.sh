#!/bin/bash

IFS=","

showHelp() {
cat << EOF  
Usage: ./gnss_attack_control.sh <options>
Control the GNSS attack system for Carla - Autoware integration

--acclon        Set the accuracy for the longitude (x axis)(meters)
--acclat        Set the accuracy for the latitude  (y axis)(meters)
--accalt        Set the accuracy for the altitude  (z axis)(meters)

--percentdrop   Set the drop change for GPS messages (0.0 - 1.0)

--offlon        Set the offset for the longitude (x axis)(meters)
--offlat        Set the offset for the latitude  (y axis)(meters)
--offalt        Set the offset for the altitude  (z axis)(meters)

--driftlon      Set the drift for longitude (x axis)(meters/min)
--driftlat      Set the drift for latitude  (y axis)(meters/min)
--driftalt      Set the drift for altitude  (z axis)(meters/min)
--driftseconds  Set the amount of time the drift is activated for changes before locking in    (seconds)
--driftmeters   Set the amount of meters before the drift is locked in for the current changes (meters)

--nodriftreset  Do not reset the cumlative drift from pre-existing drifts
--driftreset    Reset the cumlative drift from pre-existing drifts (default)

--velscalex     Set the scale factor for the velocity along the x axis (ECEF coordinates)
--velscaley     Set the scale factor for the velocity along the y axis (ECEF coordinates)
--velscalez     Set the scale factor for the velocity along the z axis (ECEF coordinates)

--attack        This is a override for setting the attack type regardless of other options.

-s|--status     Prints the current status of the settings and exits
--reset         Resets the config settings to defaults and disables the attack

EOF
# EOF 
}

printSettings() {
    status=$(ros2 topic echo /attack/gnss/status --once --field config --no-daemon)

    echo "$status"
}

options=$(getopt -l "status,reset,nodriftreset,driftreset,attack:,acclat:,acclon:,accalt:,offlat:,offlon:,offalt:,driftlat:,driftlon:,driftalt:,driftseconds:,driftmeters:,percentdrop:,velscalex:,velscaley:,velscalez:" -o "s" -aq -- "$@")

if [ $? -ne 0 ]; then
    showHelp
    exit 0
fi

eval set -- "$options"

# Checks for number inputs
re='^[+-]?[0-9]+([.][0-9]+)?$'

reInt='^[0-9]+$'

driftreset=true
attack_type=0

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
--acclat) 
    shift
    if [[ $1 =~ $re ]]; then
        acclat=$1
        attack_type=$(($attack_type|0x1))
    else
        echo -e "Invalid accuracy latitude entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--acclon) 
    shift
    if [[ $1 =~ $re ]]; then
        acclon=$1
        attack_type=$(($attack_type|0x1))
    else
        echo -e "Invalid accuracy longitude entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--accalt) 
    shift
    if [[ $1 =~ $re ]]; then
        accalt=$1
        attack_type=$(($attack_type|0x1))
    else
        echo -e "Invalid accuracy altitude entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--percentdrop) 
    shift
    if [[ $1 =~ $re ]]; then
        percent_drop="$1"
        attack_type=$(($attack_type|0x2))
    else
        echo -e "Invalid percent drop entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--offlat)
    shift
    if [[ $1 =~ $re ]]; then
        offlat=$1
        attack_type=$(($attack_type|0x4))
    else
        echo -e "Invalid accuracy latitude entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--offlon)
    shift
    if [[ $1 =~ $re ]]; then
        offlon=$1
        attack_type=$(($attack_type|0x4))
    else
        echo -e "Invalid accuracy longitude entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--offalt)
    shift
    if [[ $1 =~ $re ]]; then
        offalt=$1
        attack_type=$(($attack_type|0x4))
    else
        echo -e "Invalid accuracy altitude entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--driftlat)
    shift
    if [[ $1 =~ $re ]]; then
        driftlat=$1
        attack_type=$(($attack_type|0x8))
    else
        echo -e "Invalid drift latitude entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--driftlon)
    shift
    if [[ $1 =~ $re ]]; then
        driftlon=$1
        attack_type=$(($attack_type|0x8))
    else
        echo -e "Invalid drift longitude entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--driftalt)
    shift
    if [[ $1 =~ $re ]]; then
        driftalt=$1
        attack_type=$(($attack_type|0x8))
    else
        echo -e "Invalid drift altitude entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--driftmeters)
    shift
    if [[ $1 =~ $re ]]; then
        driftmeters=$1
        attack_type=$(($attack_type|0x10))
    else
        echo -e "Invalid drift stop meters entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--driftseconds)
    shift
    if [[ $1 =~ $re ]]; then
        driftseconds=$1
        attack_type=$(($attack_type|0x20))
    else
        echo -e "Invalid drift stop seconds entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--velscalex)
    shift
    if [[ $1 =~ $re ]]; then
        velscalex=$1
        attack_type=$(($attack_type|0x40))
    else
        echo -e "Invalid velocity scale x entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--velscaley)
    shift
    if [[ $1 =~ $re ]]; then
        velscaley=$1
        attack_type=$(($attack_type|0x40))
    else
        echo -e "Invalid velocity scale y entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--velscalez)
    shift
    if [[ $1 =~ $re ]]; then
        velscalez=$1
        attack_type=$(($attack_type|0x40))
    else
        echo -e "Invalid velocity scale z entry $1\n"
        showHelp
        exit -1
    fi
    ;;
--nodriftreset)
    driftreset=false
    ;;
--driftreset)
    driftreset=true
    ;;
-s|--status)
    printSettings
    exit 0
    ;;
--reset)
    echo "Resetting GNSS Attack Config"
    ros2 topic pub --once /attack/gnss/config attack_gnss_msgs/msg/GNSSAttackConfig "attack_type: 0
accuracy_lat: 0.0
accuracy_lon: 0.0
accuracy_alt: 0.0
drop_chance: 0.0
offset_lat: 0.0
offset_lon: 0.0
offset_alt: 0.0
drift_lat: 0.0
drift_lon: 0.0
drift_alt: 0.0
drift_stop_meter: 0.0
drift_stop_second: 0.0
vel_scale_x: 1.0
vel_scale_y: 1.0
vel_scale_z: 1.0
reset_drift: true"
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

command_string=$'ros2 topic pub --once /attack/gnss/config attack_gnss_msgs/msg/GNSSAttackConfig "'

if [ ! -z $attack_type_override ]; then
    attack_type=$attack_type_override
fi

if [ $attack_type -eq 0 ] && [ -z $attack_type_override ]; then
    echo -e "No options were selected\n"
    showHelp
    exit 0
fi

command_string+=$'\nattack_type: '$attack_type

if (($attack_type&0x1)); then
    if [ ! -z "$acclat" ]; then
        command_string+=$'\naccuracy_lat: '$acclat    
    fi
    if [ ! -z "$acclon" ]; then
        command_string+=$'\naccuracy_lon: '$acclon    
    fi
    if [ ! -z "$accalt" ]; then
        command_string+=$'\naccuracy_alt: '$accalt    
    fi
fi

if (($attack_type&0x2)); then
    command_string+=$'\ndrop_chance: '$percent_drop    
fi

if (($attack_type&0x4)); then
    if [ ! -z "$offlat" ]; then
        command_string+=$'\noffset_lat: '$offlat    
    fi
    if [ ! -z "$offlon" ]; then
        command_string+=$'\noffset_lon: '$offlon    
    fi
    if [ ! -z "$offalt" ]; then
        command_string+=$'\noffset_alt: '$offalt    
    fi
fi

if (($attack_type&0x8)); then
    if [ ! -z "$driftlat" ]; then
        command_string+=$'\ndrift_lat: '$driftlat    
    fi
    if [ ! -z "$driftlon" ]; then
        command_string+=$'\ndrift_lon: '$driftlon    
    fi
    if [ ! -z "$driftalt" ]; then
        command_string+=$'\ndrift_alt: '$driftalt    
    fi
fi

if (($attack_type&0x10)); then
    command_string+=$'\ndrift_stop_meter: '$driftmeters    
fi

if (($attack_type&0x20)); then
    command_string+=$'\ndrift_stop_second: '$driftseconds
fi

if (($attack_type&0x40)); then
    if [ ! -z "$velscalex" ]; then
        command_string+=$'\nvel_scale_x: '$velscalex    
    fi
    if [ ! -z "$velscaley" ]; then
        command_string+=$'\nvel_scale_y: '$velscaley    
    fi
    if [ ! -z "$velscalez" ]; then
        command_string+=$'\nvel_scale_z: '$velscalez    
    fi
fi

command_string+=$'\nreset_drift: '$driftreset

command_string+=$'"'

eval $command_string

printSettings


#ifndef GEAR_CONVERSION_HPP
#define GEAR_CONVERSION_HPP

#include <cstdlib>
#include <tuple>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>

namespace Gears
{
//Small abuse of namespace to keep all data together for carla gears
namespace CarlaGears
{
    namespace CARLA_GEAR_DEFAULTS
    {   
        constexpr int8_t gear = 0;
        constexpr bool reverse = false;
        constexpr bool hand_brake = false;
    }

    namespace CARLA_GEAR_PARK
    {
        constexpr int8_t gear = 0;
        constexpr bool reverse = CARLA_GEAR_DEFAULTS::reverse;
        constexpr bool hand_brake = true;
    }

    namespace CARLA_GEAR_REVERSE
    {
        constexpr int8_t gear = -1;
        constexpr bool reverse = true;
        constexpr bool hand_brake = CARLA_GEAR_DEFAULTS::hand_brake;
    }

    namespace CARLA_GEAR_NEUTRAL
    {
        constexpr uint8_t gear = 0;
        constexpr bool reverse = CARLA_GEAR_DEFAULTS::reverse;
        constexpr bool hand_brake = CARLA_GEAR_DEFAULTS::hand_brake;
    }

    namespace CARLA_GEAR_DRIVE
    {
        constexpr int8_t gear = 1;
        constexpr bool reverse = CARLA_GEAR_DEFAULTS::reverse;
        constexpr bool hand_brake = CARLA_GEAR_DEFAULTS::hand_brake;
    }

    namespace CARLA_GEAR_DRIVE_2
    {
        constexpr int8_t gear = 2;
        constexpr bool reverse = CARLA_GEAR_DEFAULTS::reverse;
        constexpr bool hand_brake = CARLA_GEAR_DEFAULTS::hand_brake;
    }

    namespace CARLA_GEAR_DRIVE_3
    {
        constexpr uint8_t gear = 3;
        constexpr bool reverse = CARLA_GEAR_DEFAULTS::reverse;
        constexpr bool hand_brake = CARLA_GEAR_DEFAULTS::hand_brake;
    }

    namespace CARLA_GEAR_DRIVE_4
    {
        constexpr int8_t gear = 4;
        constexpr bool reverse = CARLA_GEAR_DEFAULTS::reverse;
        constexpr bool hand_brake = CARLA_GEAR_DEFAULTS::hand_brake;
    }

    namespace CARLA_GEAR_DRIVE_5
    {
        constexpr int8_t gear = 5;
        constexpr bool reverse = CARLA_GEAR_DEFAULTS::reverse;
        constexpr bool hand_brake = CARLA_GEAR_DEFAULTS::hand_brake;
    }

    namespace CARLA_GEAR_DRIVE_6
    {
        constexpr int8_t gear = 6;
        constexpr bool reverse = CARLA_GEAR_DEFAULTS::reverse;
        constexpr bool hand_brake = CARLA_GEAR_DEFAULTS::hand_brake;
    }

    namespace CARLA_GEAR_DRIVE_7
    {
        constexpr int8_t gear = 7;
        constexpr bool reverse = CARLA_GEAR_DEFAULTS::reverse;
        constexpr bool hand_brake = CARLA_GEAR_DEFAULTS::hand_brake;
    }

    namespace CARLA_GEAR_DRIVE_8
    {
        constexpr int8_t gear = 8;
        constexpr bool reverse = CARLA_GEAR_DEFAULTS::reverse;
        constexpr bool hand_brake = CARLA_GEAR_DEFAULTS::hand_brake;
    }

    namespace CARLA_GEAR_DRIVE_9
    {
        constexpr int8_t gear = 9;
        constexpr bool reverse = CARLA_GEAR_DEFAULTS::reverse;
        constexpr bool hand_brake = CARLA_GEAR_DEFAULTS::hand_brake;
    }
}
using namespace CarlaGears;

enum PastaGears : int
{
    PASTA_GEAR_NONE = 0,
    PASTA_GEAR_PARK = 1,
    PASTA_GEAR_REVERSE = 2,
    PASTA_GEAR_NEUTRAL = 3,
    PASTA_GEAR_DRIVE = 4,
    PASTA_GEAR_LOW = 5
};

//Returns pair that is the gear, reverse, handbrake
inline static std::tuple<int32_t, bool, bool> autoware_2_carla_gears(autoware_auto_vehicle_msgs::msg::GearCommand::_command_type autowareGear)
{
    using AWGear = autoware_auto_vehicle_msgs::msg::GearCommand;

    bool reverse = CARLA_GEAR_DEFAULTS::reverse;
    bool hand_brake = CARLA_GEAR_DEFAULTS::hand_brake;
    int32_t gear = CARLA_GEAR_DEFAULTS::gear;

    switch(autowareGear)
    {
        case AWGear::DRIVE:
        gear = CARLA_GEAR_DRIVE::gear;
        reverse = CARLA_GEAR_DRIVE::reverse;
        hand_brake = CARLA_GEAR_DRIVE::hand_brake;
        break;
        case AWGear::DRIVE_2:
        gear = CARLA_GEAR_DRIVE_2::gear;
        reverse = CARLA_GEAR_DRIVE_2::reverse;
        hand_brake = CARLA_GEAR_DRIVE_2::hand_brake;
        break;
        case AWGear::DRIVE_3:
        gear = CARLA_GEAR_DRIVE_3::gear;
        reverse = CARLA_GEAR_DRIVE_3::reverse;
        hand_brake = CARLA_GEAR_DRIVE_3::hand_brake;
        break;
        case AWGear::DRIVE_4:
        gear = CARLA_GEAR_DRIVE_4::gear;
        reverse = CARLA_GEAR_DRIVE_4::reverse;
        hand_brake = CARLA_GEAR_DRIVE_4::hand_brake;
        break;
        case AWGear::DRIVE_5:
        gear = CARLA_GEAR_DRIVE_5::gear;
        reverse = CARLA_GEAR_DRIVE_5::reverse;
        hand_brake = CARLA_GEAR_DRIVE_5::hand_brake;
        break;
        case AWGear::DRIVE_6:
        gear = CARLA_GEAR_DRIVE_6::gear;
        reverse = CARLA_GEAR_DRIVE_6::reverse;
        hand_brake = CARLA_GEAR_DRIVE_6::hand_brake;
        break;
        case AWGear::DRIVE_7:
        gear = CARLA_GEAR_DRIVE_7::gear;
        reverse = CARLA_GEAR_DRIVE_7::reverse;
        hand_brake = CARLA_GEAR_DRIVE_7::hand_brake;
        break;
        case AWGear::DRIVE_8:
        gear = CARLA_GEAR_DRIVE_8::gear;
        reverse = CARLA_GEAR_DRIVE_8::reverse;
        hand_brake = CARLA_GEAR_DRIVE_8::hand_brake;
        break;
        case AWGear::DRIVE_9:
        gear = CARLA_GEAR_DRIVE_9::gear;
        reverse = CARLA_GEAR_DRIVE_9::reverse;
        hand_brake = CARLA_GEAR_DRIVE_9::hand_brake;
        break;
        case AWGear::REVERSE:
        gear = CARLA_GEAR_REVERSE::gear;
        reverse = CARLA_GEAR_REVERSE::reverse;
        hand_brake = CARLA_GEAR_REVERSE::hand_brake;
        break;
        case AWGear::NEUTRAL:
        gear = CARLA_GEAR_NEUTRAL::gear;
        reverse = CARLA_GEAR_NEUTRAL::reverse;
        hand_brake = CARLA_GEAR_NEUTRAL::hand_brake;
        break;
        case AWGear::PARK:
        gear = CARLA_GEAR_PARK::gear;
        reverse = CARLA_GEAR_PARK::reverse;
        hand_brake = CARLA_GEAR_PARK::hand_brake;
        break;
        case AWGear::NONE:
        default:
        break;
    }

    return std::make_tuple(gear, reverse, hand_brake);
}

inline static autoware_auto_vehicle_msgs::msg::GearCommand::_command_type carla_2_autoware_gears(int32_t gear, bool reverse, bool hand_brake)
{
    using AWGear = autoware_auto_vehicle_msgs::msg::GearCommand;

    autoware_auto_vehicle_msgs::msg::GearCommand::_command_type gearRet;

    if (reverse)
    {
        gearRet = AWGear::REVERSE;
    }
    else
    {
        switch(gear)
        {
        case CARLA_GEAR_NEUTRAL::gear:
            if (hand_brake)
            {
                gearRet = AWGear::PARK;
            }
            else
            {
                gearRet = AWGear::NEUTRAL;
            }
            break;
        case CARLA_GEAR_REVERSE::gear:
            gearRet = AWGear::REVERSE;
            break;
        case CARLA_GEAR_DRIVE::gear:
            gearRet = AWGear::DRIVE;
            break;
        case CARLA_GEAR_DRIVE_2::gear:
            gearRet = AWGear::DRIVE_2;
            break;
        case CARLA_GEAR_DRIVE_3::gear:
            gearRet = AWGear::DRIVE_3;
            break;
        case CARLA_GEAR_DRIVE_4::gear:
            gearRet = AWGear::DRIVE_4;
            break;
        case CARLA_GEAR_DRIVE_5::gear:
            gearRet = AWGear::DRIVE_5;
            break;
        case CARLA_GEAR_DRIVE_6::gear:
            gearRet = AWGear::DRIVE_6;
            break;
        case CARLA_GEAR_DRIVE_7::gear:
            gearRet = AWGear::DRIVE_7;
            break;
        case CARLA_GEAR_DRIVE_8::gear:
            gearRet = AWGear::DRIVE_8;
            break;
        case CARLA_GEAR_DRIVE_9::gear:
            gearRet = AWGear::DRIVE_9;
            break;  
        default:
            gearRet = AWGear::NONE;
            break;
        }
    }

    return gearRet;
}

//Returns pair that is the gear, handbrake
inline static int32_t autoware_2_pasta_gears(autoware_auto_vehicle_msgs::msg::GearCommand::_command_type autowareGear)
{
    using AWGear = autoware_auto_vehicle_msgs::msg::GearCommand;

    int32_t gear = 0;

    switch(autowareGear)
    {
        case AWGear::LOW:
        case AWGear::LOW_2:
        gear = PASTA_GEAR_LOW;
        break;
        case AWGear::DRIVE:
        case AWGear::DRIVE_2:
        case AWGear::DRIVE_3:
        case AWGear::DRIVE_4:
        case AWGear::DRIVE_5:
        case AWGear::DRIVE_6:
        case AWGear::DRIVE_7:
        case AWGear::DRIVE_8:
        case AWGear::DRIVE_9:
        case AWGear::DRIVE_10:
        case AWGear::DRIVE_11:
        case AWGear::DRIVE_12:
        case AWGear::DRIVE_13:
        case AWGear::DRIVE_14:
        case AWGear::DRIVE_15:
        case AWGear::DRIVE_16:
        case AWGear::DRIVE_17:
        case AWGear::DRIVE_18:
        gear = PASTA_GEAR_DRIVE;
        break;
        case AWGear::REVERSE:
        case AWGear::REVERSE_2:
        gear = PASTA_GEAR_REVERSE;
        break;
        case AWGear::NEUTRAL:
        gear = PASTA_GEAR_NEUTRAL;
        break;
        case AWGear::PARK:
        gear = PASTA_GEAR_PARK;
        break;
        case AWGear::NONE:
        default:
        gear = PASTA_GEAR_NONE;
        break;
    }

    return gear;
}

inline static autoware_auto_vehicle_msgs::msg::GearCommand::_command_type pasta_2_autoware_gears(int32_t pastaGear)
{
    autoware_auto_vehicle_msgs::msg::GearCommand::_command_type autowareGear;

    switch(pastaGear)
    {
        
        case PASTA_GEAR_PARK:
            autowareGear = autoware_auto_vehicle_msgs::msg::GearCommand::PARK;
            break;
        case PASTA_GEAR_REVERSE:
            autowareGear = autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE;
            break;
        case PASTA_GEAR_NEUTRAL:
            autowareGear = autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL;
            break;
        case PASTA_GEAR_DRIVE:
            autowareGear = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE;
            break;
        case PASTA_GEAR_LOW:
            autowareGear = autoware_auto_vehicle_msgs::msg::GearCommand::LOW;
            break;
        case PASTA_GEAR_NONE:
        default:
            autowareGear = autoware_auto_vehicle_msgs::msg::GearCommand::NONE;
            break;
    }

    return autowareGear;
}

}

#endif
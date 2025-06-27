#pragma once
#include <cstdint>
#include <string>

// TODO maybe use something like this to make these nicer? https://github.com/Neargye/magic_enum
// for now going to do simple to get base as think we will have a maintainable ~10 total types but worth quick glance
// is ^ seems good header only less maitenence etc.
enum class module_lang {
    Python,
    Cpp,
    UNKNOWN,
};
inline std::string ToString(const module_lang l) {
    switch(l) {
        case module_lang::Python : return "Python";
        case module_lang::Cpp    : return "C++";
        case module_lang::UNKNOWN: return "Unknown module language";
    }
    return "Unknown module language";
};

enum class module_type {
    Test_Module,
    CAN_Data,
    AutoEther,
    Sensor_GNSS,
    Sensor_LiDAR,
    Sensor_Radar,
    Sensor_CMOS,
    Sensor_IMU,
    UNKNOWN,
};
inline std::string ToString(const module_type t) {
    switch(t) {
        case module_type::Test_Module : return "Test Module";
        case module_type::CAN_Data    : return "CAN Data";
        case module_type::AutoEther   : return "Automotive Ethernet";
        case module_type::Sensor_GNSS : return "GNSS Sensor";
        case module_type::Sensor_LiDAR: return "LiDAR Sensor";
        case module_type::Sensor_Radar: return "Radar Sensor";
        case module_type::Sensor_CMOS : return "CMOS Sensor";
        case module_type::Sensor_IMU  : return "IMU Sensor";
        case module_type::UNKNOWN     : return "Unknown module type";
    }
    return "Unknown module type";
};

extern "C" {
    struct module_info {
        module_lang language;
        module_type type;
        uint8_t     api_version;
        uint8_t     module_version;
        std::string name;
        std::string description;
        module_info( 
                     module_lang _language       = module_lang::UNKNOWN      
                   , module_type _type           = module_type::UNKNOWN  
                   , uint8_t     _api_version    = 0         
                   , uint8_t     _module_version = 0            
                   , std::string _name           = ""  
                   , std::string _description    = ""         
                ): language(_language)
                 , type(_type)
                 , api_version(_api_version)
                 , module_version(_module_version)
                 , name(_name)
                 , description(_description)
        {};
        friend std::ostream& operator<<(std::ostream& os, module_info const & info) {
            std::string output = "";
            output += "Name:           " + info.name                           + '\n';
            output += "Description:    " + info.description                    + '\n';
            output += "Type:           " + ToString(info.type)                 + '\n';
            output += "Language:       " + ToString(info.language)             + '\n';
            output += "API Version:    " + std::to_string(info.api_version)    + '\n';
            output += "Module Version: " + std::to_string(info.module_version) + '\n';
            return os << output;
        }
    };
}
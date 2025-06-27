#ifndef ATTACH_LIDAR_BLIND_CONFIG_HPP
#define ATTACH_LIDAR_BLIND_CONFIG_HPP

#include <attack_gnss_msgs/msg/gnss_attack_config.hpp>

#include <random>
#include <stdlib.h>

#include <mutex>

class GNSSAttackConfig {
    public:
    GNSSAttackConfig() { srand(time(NULL)); };

    inline attack_gnss_msgs::msg::GNSSAttackConfig::_attack_type_type attack_type() { return data->attack_type; };
    inline attack_gnss_msgs::msg::GNSSAttackConfig::_accuracy_lat_type accuracy_lat() { return data->accuracy_lat; };
    inline attack_gnss_msgs::msg::GNSSAttackConfig::_accuracy_lon_type accuracy_lon() { return data->accuracy_lon; };
    inline attack_gnss_msgs::msg::GNSSAttackConfig::_accuracy_alt_type accuracy_alt() { return data->accuracy_alt; };

    inline attack_gnss_msgs::msg::GNSSAttackConfig::_drop_chance_type drop_chance() { return data->drop_chance; };
    
    inline attack_gnss_msgs::msg::GNSSAttackConfig::_offset_lat_type offset_lat() { return data->offset_lat; };
    inline attack_gnss_msgs::msg::GNSSAttackConfig::_offset_lon_type offset_lon() { return data->offset_lon; };
    inline attack_gnss_msgs::msg::GNSSAttackConfig::_offset_alt_type offset_alt() { return data->offset_alt; };

    inline attack_gnss_msgs::msg::GNSSAttackConfig::_drift_lat_type drift_lat() { return data->drift_lat; };
    inline attack_gnss_msgs::msg::GNSSAttackConfig::_drift_lon_type drift_lon() { return data->drift_lon; };
    inline attack_gnss_msgs::msg::GNSSAttackConfig::_drift_alt_type drift_alt() { return data->drift_alt; };
    inline attack_gnss_msgs::msg::GNSSAttackConfig::_drift_stop_meter_type drift_stop_meter() { return data->drift_stop_meter; };
    inline attack_gnss_msgs::msg::GNSSAttackConfig::_drift_stop_second_type drift_stop_second() { return data->drift_stop_second; };

    inline attack_gnss_msgs::msg::GNSSAttackConfig::_vel_scale_x_type vel_scale_x() { return data->vel_scale_x; };
    inline attack_gnss_msgs::msg::GNSSAttackConfig::_vel_scale_y_type vel_scale_y() { return data->vel_scale_y; };
    inline attack_gnss_msgs::msg::GNSSAttackConfig::_vel_scale_z_type vel_scale_z() { return data->vel_scale_z; };

    inline attack_gnss_msgs::msg::GNSSAttackConfig::_reset_drift_type reset_drift() { return data->reset_drift; };

    void set_data(attack_gnss_msgs::msg::GNSSAttackConfig::SharedPtr newData);
    inline attack_gnss_msgs::msg::GNSSAttackConfig get_data() { return *data; }

    inline double get_random_percent() { return ((double) rand() / (RAND_MAX)); };

    double drift_start = -1;
    double drift_stop = -1;
    bool drift_run = true;

    double drift_last_run;
    int64_t drift_distance_x;
    int64_t drift_distance_y;
    int64_t drift_distance_z;

    double drift_lat_s;
    double drift_lat_cumlative = 0.0;
    double drift_lon_s;
    double drift_lon_cumlative = 0.0;
    double drift_alt_s;
    double drift_alt_cumlative = 0.0;

    std::string map_type = "unknown";
    std::string map_mgrs_grid;

    int coordinate_system = 3;
    int height_system = 0;
    int plane = 0;

    double origin_lat;
    double origin_lon;
    double origin_alt;

    int64_t processed = 0;
    int64_t dropped = 0;

    std::mutex config_access;

    private:
    std::shared_ptr<attack_gnss_msgs::msg::GNSSAttackConfig> data = std::make_shared<attack_gnss_msgs::msg::GNSSAttackConfig>();
};

#endif

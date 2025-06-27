#ifndef ATTACH_LIDAR_BLIND_CONFIG_HPP
#define ATTACH_LIDAR_BLIND_CONFIG_HPP

#include <attack_lidar_blind_msgs/msg/lidar_attack_config.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <geometry_msgs/msg/point.hpp>

#include <random>
#include <stdlib.h>

class LidarBlindAttackConfig {
    public:
    LidarBlindAttackConfig() { srand(time(NULL)); };

    inline attack_lidar_blind_msgs::msg::LidarAttackConfig::_attack_type_type attack_type() { return data->attack_type; };
    inline attack_lidar_blind_msgs::msg::LidarAttackConfig::_distance_less_type distance_less() { return data->distance_less; };
    inline attack_lidar_blind_msgs::msg::LidarAttackConfig::_distance_greater_type distance_greater() { return data->distance_greater; };
    inline attack_lidar_blind_msgs::msg::LidarAttackConfig::_distance_dropoff_type distance_dropoff() { return data->distance_dropoff; };
    inline attack_lidar_blind_msgs::msg::LidarAttackConfig::_attack_angle_type angle() { return data->attack_angle; };
    inline attack_lidar_blind_msgs::msg::LidarAttackConfig::_attack_angle_type angle_h() { return _angle_h; }
    inline attack_lidar_blind_msgs::msg::LidarAttackConfig::_attack_angle_dropoff_type angle_dropoff() { return data->attack_angle_dropoff; }
    inline attack_lidar_blind_msgs::msg::LidarAttackConfig::_z_min_type z_min() { return data->z_min; };
    inline attack_lidar_blind_msgs::msg::LidarAttackConfig::_z_min_type z_tolerance() { return data->z_tolerance; };
    inline attack_lidar_blind_msgs::msg::LidarAttackConfig::_random_drop_percent_type random_drop_percent() { return data->random_drop_percent; };
    
    const static int blend_size = 100;

    inline bool angle_blend_enabled() { return _angle_blend_enabled; }
    inline float angle_blend_start() { return _angle_blend_start; }
    inline float angle_blend_bias() { return _angle_blend_bias; }
    inline float angle_blend_unit() { return _angle_blend_unit; }
    float angle_blend_lookup(int x);

    inline bool distance_blend_enabled() { return _distance_blend_enabled; }
    inline float distance_blend_start() { return _distance_blend_start; }
    inline float distance_blend_bias() { return _distance_blend_bias; }
    inline float distance_blend_unit() { return _distance_blend_unit; }
    float distance_blend_lookup(int x);

    inline attack_lidar_blind_msgs::msg::LidarAttackConfig::_lanelet_id_type lanelet_id() { return data->lanelet_id; };

    inline geometry_msgs::msg::Point::_x_type bound_x_upper() { return _bound_upper_x; }
    inline geometry_msgs::msg::Point::_x_type bound_x_lower() { return _bound_lower_x; }
    inline geometry_msgs::msg::Point::_x_type bound_y_upper() { return _bound_upper_y; }
    inline geometry_msgs::msg::Point::_x_type bound_y_lower() { return _bound_lower_y; }
    inline geometry_msgs::msg::Point::_x_type bound_z_upper() { return _bound_upper_z; }
    inline geometry_msgs::msg::Point::_x_type bound_z_lower() { return _bound_lower_z; }
    inline bool bound_x_valid() { return _bound_x_valid; }
    inline bool bound_y_valid() { return _bound_y_valid; }
    inline bool bound_z_valid() { return _bound_z_valid; }

    void set_data(attack_lidar_blind_msgs::msg::LidarAttackConfig::SharedPtr newData);
    inline attack_lidar_blind_msgs::msg::LidarAttackConfig get_data() { return *data; }

    inline double get_random_percent() { return ((double) rand() / (RAND_MAX)); };

    lanelet::ConstLanelets lanelets;
    std::shared_ptr<lanelet::ConstLanelet> attack_lanelet;

    private:
    std::shared_ptr<attack_lidar_blind_msgs::msg::LidarAttackConfig> data = std::make_shared<attack_lidar_blind_msgs::msg::LidarAttackConfig>();
    attack_lidar_blind_msgs::msg::LidarAttackConfig::_attack_angle_type _angle_h = 0;

    geometry_msgs::msg::Point::_x_type _bound_lower_x, _bound_upper_x;
    geometry_msgs::msg::Point::_y_type _bound_lower_y, _bound_upper_y;
    geometry_msgs::msg::Point::_z_type _bound_lower_z, _bound_upper_z;

    bool _bound_x_valid = false;
    bool _bound_y_valid = false;
    bool _bound_z_valid = false;

    bool _angle_blend_enabled = false;
    float _angle_blend_bias;
    float _angle_blend_start;
    float _angle_blend_unit;
    float _angle_blend_lookup[100];

    bool _distance_blend_enabled = false;
    float _distance_blend_bias;
    float _distance_blend_start;
    float _distance_blend_unit;
    float _distance_blend_lookup[100];
};

#endif
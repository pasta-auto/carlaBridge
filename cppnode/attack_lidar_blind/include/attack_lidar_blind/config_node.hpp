#ifndef ATTACH_LIDAR_BLIND_CONFIG_NODE_HPP
#define ATTACH_LIDAR_BLIND_CONFIG_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <attack_lidar_blind_msgs/msg/lidar_attack_config.hpp>

#include "attack_lidar_blind/config.hpp"

class LidarBlindAttackConfigNode : public rclcpp::Node
{
public:
    LidarBlindAttackConfigNode();

    void config_callback(attack_lidar_blind_msgs::msg::LidarAttackConfig::UniquePtr msg);
    void mapbin_callback(autoware_auto_mapping_msgs::msg::HADMapBin::UniquePtr msg);

    inline std::shared_ptr<LidarBlindAttackConfig> get_config() { return config; }

private:
    rclcpp::Subscription<attack_lidar_blind_msgs::msg::LidarAttackConfig>::SharedPtr config_sub;
    rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_bin_sub;

    std::shared_ptr<LidarBlindAttackConfig> config = std::make_shared<LidarBlindAttackConfig>();
};

#endif
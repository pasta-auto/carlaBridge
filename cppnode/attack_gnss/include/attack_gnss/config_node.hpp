#ifndef ATTACH_GNSS_CONFIG_NODE_HPP
#define ATTACH_GNSS_CONFIG_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <attack_gnss_msgs/msg/gnss_attack_config.hpp>

#include "attack_gnss/config.hpp"

class GNSSAttackConfigNode : public rclcpp::Node
{
public:
    GNSSAttackConfigNode();

    void config_callback(attack_gnss_msgs::msg::GNSSAttackConfig::UniquePtr msg);

    inline std::shared_ptr<GNSSAttackConfig> get_config() { return config; }

private:
    rclcpp::Subscription<attack_gnss_msgs::msg::GNSSAttackConfig>::SharedPtr config_sub;

    std::shared_ptr<GNSSAttackConfig> config = std::make_shared<GNSSAttackConfig>();
};

#endif
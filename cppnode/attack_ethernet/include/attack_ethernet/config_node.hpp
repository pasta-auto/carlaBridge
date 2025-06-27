#ifndef ATTACK_ETHERNET_CONFIG_NODE_HPP
#define ATTACK_ETHERNET_CONFIG_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <attack_ethernet_msgs/msg/ethernet_attack_config.hpp>

#include "attack_ethernet/config.hpp"

class EthernetAttackConfigNode : public rclcpp::Node
{
public:
    EthernetAttackConfigNode();

    void config_callback(attack_ethernet_msgs::msg::EthernetAttackConfig::UniquePtr msg);

    inline std::shared_ptr<EthernetAttackConfig> get_config() { return config; }

private:
    rclcpp::Subscription<attack_ethernet_msgs::msg::EthernetAttackConfig>::SharedPtr config_sub;

    std::shared_ptr<EthernetAttackConfig> config = std::make_shared<EthernetAttackConfig>();
};

#endif

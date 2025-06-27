#include "attack_ethernet/config.hpp"
#include "attack_ethernet/config_node.hpp"

static rclcpp::Node* node_ptr; 

using std::placeholders::_1;

EthernetAttackConfigNode::EthernetAttackConfigNode() : Node("attack_ethernet_config", rclcpp::NodeOptions().use_intra_process_comms(false))
{
    node_ptr = this;
    
    std::string config_path = this->declare_parameter("config", "/attack/ethernet/config");

    config_sub = this->create_subscription<attack_ethernet_msgs::msg::EthernetAttackConfig>(
        config_path, rclcpp::SensorDataQoS(), std::bind(&EthernetAttackConfigNode::config_callback, this, _1));
}

void EthernetAttackConfigNode::config_callback(attack_ethernet_msgs::msg::EthernetAttackConfig::UniquePtr msg)
{
    std::lock_guard<std::mutex> guard(config->config_access);

    //move config
    config->set_data(std::move(msg));
}

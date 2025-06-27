#ifndef ATTACK_ETHERNET_CONFIG_HPP
#define ATTACK_ETHERNET_CONFIG_HPP

#include <attack_ethernet_msgs/msg/ethernet_attack_config.hpp>

#include <random>
#include <stdlib.h>

#include <mutex>

class EthernetAttackConfig {
    public:
    EthernetAttackConfig() { srand(time(NULL)); };

    inline attack_ethernet_msgs::msg::EthernetAttackConfig::_attack_type_type attack_type() { return data->attack_type; };
    inline attack_ethernet_msgs::msg::EthernetAttackConfig::_attack_target_type attack_target() { return data->attack_target; };

    inline attack_ethernet_msgs::msg::EthernetAttackConfig::_replay_count_type replay_count() { return data->replay_count; };
    inline attack_ethernet_msgs::msg::EthernetAttackConfig::_replay_delay_type replay_delay() { return data->replay_delay; };
    inline attack_ethernet_msgs::msg::EthernetAttackConfig::_replay_percent_type replay_percent() { return data->replay_percent; };

    inline attack_ethernet_msgs::msg::EthernetAttackConfig::_dos_delay_type dos_delay() { return data->dos_delay; };
    inline attack_ethernet_msgs::msg::EthernetAttackConfig::_dos_drop_percent_type dos_drop_percent() { return data->dos_drop_percent; };

    inline attack_ethernet_msgs::msg::EthernetAttackConfig::_dos_count_type dos_count() { return data->dos_count; };

    void set_data(attack_ethernet_msgs::msg::EthernetAttackConfig::SharedPtr newData) { data = newData; }
    inline attack_ethernet_msgs::msg::EthernetAttackConfig get_data() { return *data; }

    inline double get_random_percent() { return ((double) rand() / (RAND_MAX)); };

    std::mutex config_access;

    private:
    std::shared_ptr<attack_ethernet_msgs::msg::EthernetAttackConfig> data = std::make_shared<attack_ethernet_msgs::msg::EthernetAttackConfig>();
};

#endif

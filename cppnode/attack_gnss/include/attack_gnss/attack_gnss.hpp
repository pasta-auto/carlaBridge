#ifndef ATTACK_GNSS_HPP
#define ATTACK_GNSS_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <attack_gnss_msgs/msg/gnss_attack_status.hpp>
#include <attack_gnss_msgs/msg/gnss_attack_config.hpp>

#include <attack_gnss/config.hpp>

class GNSSAttackNode : public rclcpp::Node
{
public: 
    GNSSAttackNode(std::shared_ptr<GNSSAttackConfig> config);

    void navsat_callback(sensor_msgs::msg::NavSatFix::UniquePtr msg);
    void gnss_vel_callback(geometry_msgs::msg::TwistWithCovarianceStamped::UniquePtr msg);

private:
    std::shared_ptr<GNSSAttackConfig> attack_config;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix, std::allocator<void>>> navsat_pub;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped, std::allocator<void>>> gnss_vel_pub;
    std::shared_ptr<rclcpp::Publisher<attack_gnss_msgs::msg::GNSSAttackStatus, std::allocator<void>>> status_pub;
    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gnss_vel_sub;
};

#endif

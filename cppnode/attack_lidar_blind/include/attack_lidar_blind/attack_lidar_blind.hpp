#ifndef ATTACH_LIDAR_BLIND_HPP
#define ATTACH_LIDAR_BLIND_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

#include <attack_lidar_blind_msgs/msg/lidar_blind_status.hpp>
#include <attack_lidar_blind_msgs/msg/lidar_attack_config.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <attack_lidar_blind/config.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class LidarBlindAttackNode : public rclcpp::Node
{
public: 
    LidarBlindAttackNode(std::shared_ptr<LidarBlindAttackConfig> config);
    void lidar_callback(sensor_msgs::msg::PointCloud2::UniquePtr msg);

private:
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    std::shared_ptr<LidarBlindAttackConfig> attack_config;

    const std::string azimuth_field_name = "azimuth";
    const std::string distance_field_name = "distance";
    const std::string x_field_name = "x";
    const std::string y_field_name = "y";
    const std::string z_field_name = "z";

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2, std::allocator<void>>> lidar_pub;
    std::shared_ptr<rclcpp::Publisher<attack_lidar_blind_msgs::msg::LidarBlindStatus, std::allocator<void>>> status_pub;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub;
    
};

#endif
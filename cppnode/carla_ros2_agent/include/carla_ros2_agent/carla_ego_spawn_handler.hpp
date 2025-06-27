#ifndef CARLA_EGO_SPAWN_HANDLER_HPP
#define CARLA_EGO_SPAWN_HANDLER_HPP

#include <carla/client/World.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <dummy_perception_publisher/msg/object.hpp>
#include <geometry_msgs/msg/pose.hpp>

class carla_ego_spawn_handler : public rclcpp::Node
{
using json = nlohmann::json;

public:     
    const std::string spawn_role_name = "bridge_spawn_actor";

    carla_ego_spawn_handler();
    void set_world(std::weak_ptr<carla::client::World> world_ptr);
    void spawn_ego(std::string spawn_objects_path);
    void destroy_ego(const std::string& role_name);

private:
    std::weak_ptr<carla::client::World> world_ptr;
    rclcpp::Subscription<dummy_perception_publisher::msg::Object>::SharedPtr spawn_object_sub;
    bool spawn_actor_json(json actor_json, std::shared_ptr<carla::client::World> world, carla::SharedPtr<carla::client::Actor> parent = carla::SharedPtr<carla::client::Actor>());
    void spawn_actor_object(int type, geometry_msgs::msg::Pose pose);

    void spawn_handler(dummy_perception_publisher::msg::Object::UniquePtr msg);
};

#endif
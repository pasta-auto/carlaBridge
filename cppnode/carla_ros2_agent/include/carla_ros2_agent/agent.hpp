#ifndef AGENT_HPP
#define AGENT_HPP

#include <carla_ros2_agent/autoware_ros2_agent.hpp>

#include <carla/client/World.h>

#include <carla_ros2_agent/carla_ros2_agent.hpp>
#include <carla_ros2_agent/autoware_ros2_agent.hpp>
#include <carla_ros2_agent/carla_ego_spawn_handler.hpp>

#include <memory>
#include <thread>

namespace CarlaAutowareRos2
{

class Agent
{
public:
    Agent();
    void start();
    void stop();
    void prep();
    void run_step();
    void set_world(std::weak_ptr<carla::client::World>);

    std::string get_carla_ip();
    int get_carla_port();
    int get_frame_rate();
    std::string get_agent_role_name();
    std::string get_spawn_object_path();

private:
    std::weak_ptr<carla::client::World> world_ptr;
    carla::SharedPtr<carla::client::Actor> spectator;
    carla::client::Timestamp timestamp_last_run;

    std::shared_ptr<AutowareAgent> autoware_node;
    std::shared_ptr<CarlaAgent> carla_node;
    std::shared_ptr<carla_ego_spawn_handler> spawn_node;
    rclcpp::executors::MultiThreadedExecutor executor;
    std::thread node_spin;
};
}

#endif
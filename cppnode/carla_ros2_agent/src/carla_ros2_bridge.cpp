#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <carla_ros2_agent/world_handler.hpp>
#include <carla_ros2_agent/agent_handler.hpp>
#include <carla_ros2_agent/carla_ego_spawn_handler.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto agent_h = agent_handler();

    //Setup world
    world_handler world = world_handler();

    world.set_frametime(1.0 / (double)agent_h.agent->get_frame_rate());
    auto ip = agent_h.agent->get_carla_ip();
    auto port = agent_h.agent->get_carla_port();

    if (!world.load_world(ip, port))
    {
        return -1;
    }

    agent_h.set_frametime(world.get_frametime());
    agent_h.set_world(world.get_world());
    agent_h.run_agent();

  return 0;
}
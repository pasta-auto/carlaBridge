#ifndef AGENT_HANDLER_HPP
#define AGENT_HANDLER_HPP

#include <carla/client/World.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <carla_ros2_agent/agent.hpp>

class agent_handler
{
public:
    agent_handler();
    void set_world(std::weak_ptr<carla::client::World> world_ptr);
    void set_frametime(double rate);
    void run_agent();
    void stop_agent();

    std::unique_ptr<CarlaAutowareRos2::Agent> agent;

private:
    std::shared_ptr<rclcpp::Node> node;

    std::mutex frame_lock;
    std::condition_variable frame_cv;

    std::atomic_bool pause = false;
    bool running = false;
    bool rate_change = true;

    const int64_t adaptive_change_limit = 10000;
    int64_t adaptive_hz_change = 0;
    std::chrono::microseconds  adaptive_frametime;
    std::chrono::microseconds  base_frametime;
    std::chrono::microseconds  configured_frametime;
    double frametime_hz;
    uint32_t frametime_hz_count;

    std::thread agent_thread;
    std::thread frametime_thread;
    std::thread carla_rate_thread;
    std::thread ros2TimerLoop_thread;
    void carla_rate_spin();
    std::shared_ptr<rclcpp::Executor> node_executor;

    std::weak_ptr<carla::client::World> world_ptr;
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr on_pause_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr on_rate_sub;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32, std::allocator<void>>> on_hz_pub;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool, std::allocator<void>>> is_paused_pub;

    std::shared_ptr<rclcpp::WallTimer<std::_Bind<void (agent_handler::*(agent_handler *))()>, (void *)nullptr>> hz_pub_timer;

    void frametime_lock_loop();
    void agent_loop();

    void onPause(std_msgs::msg::Bool::UniquePtr data);
    void onRate(std_msgs::msg::Float32::UniquePtr data);
    void ros2TimerLoop();
};

#endif
#include <carla_ros2_agent/agent_handler.hpp>
#include <carla_ros2_agent/carla_ros2_agent.hpp>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <thread>

using namespace std::chrono_literals;

agent_handler::agent_handler() 
{
    agent = std::make_unique<CarlaAutowareRos2::Agent>();

    //Setup control node for clock
    rclcpp::NodeOptions options;
    // options.use_clock_thread(false);
    node = std::make_shared<rclcpp::Node>("CarlaClock", "/carla", options);
    node_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    on_pause_sub = node->create_subscription<std_msgs::msg::Bool>("pause", rclcpp::QoS(1), std::bind(&agent_handler::onPause, this, std::placeholders::_1));
    on_rate_sub = node->create_subscription<std_msgs::msg::Float32>("rate", rclcpp::QoS(1), std::bind(&agent_handler::onRate, this, std::placeholders::_1));
    
    on_hz_pub = node->create_publisher<std_msgs::msg::Float32>("current_hz", rclcpp::QoS(1));
    is_paused_pub = node->create_publisher<std_msgs::msg::Bool>("is_paused", rclcpp::QoS(1));
}

void agent_handler::onPause(std_msgs::msg::Bool::UniquePtr data)
{
    pause = data->data;

    std_msgs::msg::Bool pause_msg;
    pause_msg.data = pause;
    is_paused_pub->publish(pause_msg);
}

void agent_handler::onRate(std_msgs::msg::Float32::UniquePtr data)
{
    rate_change = true;
    configured_frametime = std::chrono::microseconds((uint32_t)(base_frametime.count() / data->data));

    adaptive_frametime = configured_frametime;
    adaptive_hz_change = 0;

    frametime_hz = 0.0;
    frametime_hz_count = 0;
}

void agent_handler::ros2TimerLoop()
{   
    static bool was_paused = true;

    while(running)
    {
        std::this_thread::sleep_for(2s);

        std_msgs::msg::Bool pause_msg;
        pause_msg.data = pause;
        is_paused_pub->publish(pause_msg);

        if (pause)
        {
            was_paused = true;
            continue;
        }
        else 
        {
            if (was_paused)
            {
                was_paused = false;

                frametime_hz = 0.0;
                frametime_hz_count = 0;

                continue;
            }
        }

        double frametime_accu = frametime_hz;
        uint32_t frametime_count = frametime_hz_count;

        frametime_hz = 0.0;
        frametime_hz_count = 0;

        double frametime_averaged = frametime_accu / frametime_count;

        std_msgs::msg::Float32 hz_msg;
        
        hz_msg.data = 1 / frametime_averaged;

        int64_t adaptive_hz_diff = configured_frametime.count() - std::chrono::microseconds::period::den * frametime_averaged;

        if (abs(adaptive_hz_diff) > adaptive_change_limit)
        {
            if (adaptive_hz_diff < 0)
            {
                adaptive_hz_diff = -adaptive_change_limit;
            }
            else 
            {
                adaptive_hz_diff = adaptive_change_limit;
            }
        }
        adaptive_hz_change += adaptive_hz_diff;

        on_hz_pub->publish(hz_msg);

        //Skip if rate change was done to allow stableization of measurement
        if (rate_change)
        {
            rate_change = false;
            continue;
        }

        if (!(adaptive_hz_change < -10000))
        {
            adaptive_frametime = std::chrono::microseconds(configured_frametime.count() + adaptive_hz_change);
        }
    }
}

void agent_handler::set_world(std::weak_ptr<carla::client::World> newWorld)
{
    world_ptr = newWorld;

    if (agent)
    {
        agent->set_world(newWorld);
    }
}

void agent_handler::set_frametime(double seconds_rate)
{
    base_frametime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::seconds{1} * seconds_rate);
    configured_frametime = base_frametime;
    adaptive_frametime = configured_frametime;
}

void agent_handler::carla_rate_spin()
{
    node_executor->add_node(node);
    node_executor->spin();
}

void agent_handler::run_agent()
{
    running = true;

    agent->prep();

    carla_rate_thread = std::thread(&agent_handler::carla_rate_spin, this);
    frametime_thread = std::thread(&agent_handler::frametime_lock_loop, this);
    agent_thread = std::thread(&agent_handler::agent_loop, this);
    ros2TimerLoop_thread = std::thread(&agent_handler::ros2TimerLoop, this);

    agent->start();

    running = false;

    ros2TimerLoop_thread.join();
    carla_rate_thread.join();
    frametime_thread.join();
    agent_thread.join();
}

void agent_handler::agent_loop()
{
    double previous_timestamp = 0;

    std::unique_lock<std::mutex> flck(frame_lock);
    while (!world_ptr.expired() && running)
    {
        if (auto world = world_ptr.lock()) 
        {
            world->Tick(2s);

            agent->run_step();

            frametime_hz += (world->GetSnapshot().GetTimestamp().platform_timestamp - previous_timestamp);
            frametime_hz_count++;

            previous_timestamp = world->GetSnapshot().GetTimestamp().platform_timestamp;
        }

        //Will advance one tick even whan paused
        frame_cv.wait_for(flck, 60s);
    }
}

void agent_handler::frametime_lock_loop() 
{
    // std::unique_lock<std::mutex> flck(frame_lock);
    while (!world_ptr.expired() && running)
    {
        std::this_thread::sleep_for(adaptive_frametime);

        //Skip this if paused
        if (pause)
        {
            continue;
        }

        frame_cv.notify_all();
    }

    running = false;
}
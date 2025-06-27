#include <carla_ros2_agent/world_handler.hpp>
#include <exception>

world_handler::~world_handler()
{
    if (world) 
    {
        auto settings = world->GetSettings();
        settings.synchronous_mode = false;

        world->ApplySettings(settings, carla::time_duration::seconds(5));
    }
}

bool world_handler::load_world(std::string host, int port) 
{
    try 
    {
        client = std::unique_ptr<carla::client::Client>(new carla::client::Client(host, port));
        client->SetTimeout(carla::time_duration::seconds(20));

        world = std::make_shared<carla::client::World>(client->GetWorld());

        auto settings = world->GetSettings();
        settings.fixed_delta_seconds = frametime;
        settings.synchronous_mode = true;

        world->ApplySettings(settings, carla::time_duration::seconds(5));
    }
    catch(std::exception &e)
    {
        std::cerr << "Carla Error: " << e.what() << std::endl;
        return false;
    }

    return true;
}

std::weak_ptr<carla::client::World> world_handler::get_world()
{
    return std::weak_ptr<carla::client::World>(world);
}

void world_handler::set_frametime(double newFrametime)
{
    frametime = newFrametime;
}

double world_handler::get_frametime() 
{
    return frametime;
}
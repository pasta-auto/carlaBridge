#ifndef WORLD_HANDLER_HPP
#define WORLD_HANDLER_HPP

#include <carla/client/Client.h>
#include <memory>

class world_handler 
{
public:
    ~world_handler();
    bool load_world(std::string host, int port);
    std::weak_ptr<carla::client::World> get_world();
    void set_frametime(double newFrametime);
    double get_frametime();

private:
    std::unique_ptr<carla::client::Client> client;
    std::shared_ptr<carla::client::World> world;

    double frametime;
};

#endif
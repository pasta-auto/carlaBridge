#include "include/camera.hpp"

#include <iostream>
#include <thread>
int main()
{
    camera_server server("/tmp/camera.socket");

    auto thread = server.runThreaded();
    thread.join();
    std::cout << "test"<< std::endl;

    return 0;
}


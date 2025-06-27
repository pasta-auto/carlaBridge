#include "include/server.hpp"

#include <iostream>

int main()
{
    cmd_socket_server server("/tmp/test.socket");

    server.runAPI();
    std::cout << "test"<< std::endl;

    return 0;
}


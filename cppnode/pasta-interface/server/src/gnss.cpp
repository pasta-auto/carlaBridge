#include "../include/gnss.hpp"
#include "../include/remote_process.hpp"
#include <sys/socket.h>
#include <sys/un.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <algorithm>
gnss_server::gnss_server(std::string api, std::string rpc, std::string rnn): cmd_socket_server(api,rpc,rnn) {}


// TODO this is very copy paste is there some template stuff I can do where
//      gnss_server can for example only implement<gnss_cmd> problem different cmds, different params to process
void gnss_server::runRPC()
{
    if (listen(rpc_sockfd, BACKLOG) == -1) {
        std::cerr << "listen" << std::endl;
        return;
    }

    ssize_t numRead;
    gnss_cmd cmd; // TODO specifc line

    for (;;) {
        //printf("Waiting for RPC connection...\n");
        rpc_connfd = accept(rpc_sockfd, NULL, NULL);
        //printf("Accepted RPC socket fd = %d\n", rpc_connfd);
        // TODO these 2 lines are specific
        while ((numRead = read(rpc_connfd, &cmd, sizeof(gnss_cmd))) > 0) {
            if (numRead < 0 || (size_t)(numRead) < sizeof(gnss_cmd))
            {
                std::cerr << "Invalid command reveived" << std::endl;
                break;
            }

            if (cmd.rpc_socket_api_version != RPC_SOCKET_VERSION) {
                // TODO need to properly send back an error message in these
                break;
            }

            // TODO these 2 lines are specific
            double lat = cmd.data[0], lon = cmd.data[1], alt = cmd.data[2];
            process(
                lat,lon,alt
            ); 
            cmd.data[0] = lat; cmd.data[1] = lon; cmd.data[2] = alt;
            send(rpc_connfd, &cmd, sizeof(gnss_cmd), 0);

            if (close(rpc_connfd) == -1) {
                rpc_connfd = -1;
                std::cerr << "close" << std::endl;
                return;
            }
        }
    }
}

void gnss_server::GNSS_PROCESS
{
    double args[ ] = {latitude, longitude, altitude};
    try {
        char *error = NULL;
        for(auto &module: modules) {
            auto runFunc = (void (*)(double *))dlsym(module->handle, "run");
            if ((error = dlerror()) != NULL)  {
                printf("failed to find run func: %s\n", error);
            } else {
                runFunc(&args[0]);
            }
        }
        // putting input args into an array that will be manipulated by SOs that then needs to be returned
        latitude  = args[0];
        longitude = args[1];
        altitude  = args[2];
    } catch (std::exception &e) {
        std::cerr << "Could not process gnss pipeline:" <<std::endl;
        std::cerr << e.what() << std::endl;
    }
}

#ifdef WITH_MAIN
#include "../../python/include/gnss_module.hpp"
//#include <rclcpp/rclcpp.hpp>
int main(int argc, char** argv)
{
    //rclcpp::init(argc, argv);
    //auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
//
    //auto server = std::make_shared<gnss_server>();
    gnss_server server;
    auto t1 = server.runAPIThreaded();
    auto t2 = server.runRPCThreaded();
    t1.join();
    t2.join();
    //executor->add_node(server);
    //executor->spin();
    return 0;
}
#endif
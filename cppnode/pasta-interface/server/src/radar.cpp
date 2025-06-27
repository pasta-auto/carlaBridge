#include "../include/radar.hpp"
#include "../include/remote_process.hpp"
#include <sys/socket.h>
#include <sys/un.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <algorithm>
radar_server::radar_server(std::string api, std::string rpc, std::string rnn): cmd_socket_server(api,rpc,rnn) {}


// TODO this is very copy paste is there some template stuff I can do where
//      radar_server can for example only implement<radar_cmd> problem different cmds, different params to process
void radar_server::runRPC()
{
    if (listen(rpc_sockfd, BACKLOG) == -1) {
        std::cerr << "listen" << std::endl;
        return;
    }

    radar_cmd cmd; // TODO specifc line

    for (;;) {
        //printf("Waiting for RPC connection...\n");
        rpc_connfd = accept(rpc_sockfd, NULL, NULL);
        //printf("Accepted RPC socket fd = %d\n", rpc_connfd);
        // TODO these 2 lines are specific
        size_t totalNumRead = 0;
        ssize_t numRead;
        bool error = false;
        while (totalNumRead < sizeof(radar_cmd)) {
            if( (numRead = read(rpc_connfd, (char*)(&cmd)+totalNumRead, sizeof(radar_cmd)-totalNumRead)) < 0) {
                std::cerr << "Radar read error after " << totalNumRead << " bytes" << std::endl;
                error = true;
                break;
            }
            totalNumRead += numRead;
            if (numRead == 0) { //EOF
                break;
            }
        }
        if ((totalNumRead) < sizeof(radar_cmd)||error)
        {
            std::cerr << "Invalid command reveived" << std::endl;
            std::cerr << "total read " << totalNumRead << " < of sizeof(radar_cmd) expected " <<  sizeof(radar_cmd) << std::endl;
            return;
        }

        // TODO fix
        //if (cmd.rpc_socket_api_version != RPC_SOCKET_VERSION) {
        //    // TODO need to properly send back an error message in these
        //    break;
        //}

        // TODO these 2 lines are specific
        process(cmd.rawData, cmd.numPoints);
        send(rpc_connfd, &cmd, sizeof(radar_cmd), 0);

        if (close(rpc_connfd) == -1) {
            rpc_connfd = -1;
            std::cerr << "close" << std::endl;
            return;
        }
    }
}

void radar_server::RADAR_PROCESS
{
    try {
        char *error = NULL;
        for(auto &module: modules) {
            auto runFunc = (void (*)(float *, size_t))dlsym(module->handle, "run");
            if ((error = dlerror()) != NULL)  {
                printf("failed to find run func: %s\n", error);
            } else {
                runFunc(rawData, numPoints);
            }
        }
    } catch (std::exception &e) {
        std::cerr << "Could not process radar pipeline:" << std::endl;
        std::cerr << e.what() << std::endl;
    }
}

#ifdef WITH_MAIN
#include "../../python/include/radar_module.hpp"
//#include <rclcpp/rclcpp.hpp>
int main(int argc, char** argv)
{
    //rclcpp::init(argc, argv);
    //auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
//
    //auto server = std::make_shared<radar_server>();
    radar_server server;
    auto t1 = server.runAPIThreaded();
    auto t2 = server.runRPCThreaded();
    t1.join();
    t2.join();
    //executor->add_node(server);
    //executor->spin();
    return 0;
}
#endif
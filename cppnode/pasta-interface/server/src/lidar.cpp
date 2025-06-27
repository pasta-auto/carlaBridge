#include "../include/lidar.hpp"
#include "../include/remote_process.hpp"
#include <sys/socket.h>
#include <sys/un.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <algorithm>
lidar_server::lidar_server(std::string api, std::string rpc, std::string rnn): cmd_socket_server(api,rpc,rnn) {}


// TODO this is very copy paste is there some template stuff I can do where
//      lidar_server can for example only implement<lidar_cmd> problem different cmds, different params to process
void lidar_server::runRPC()
{
    if (listen(rpc_sockfd, BACKLOG) == -1) {
        std::cerr << "listen" << std::endl;
        return;
    }

    lidar_cmd cmd; // TODO specifc line

    for (;;) {
        //printf("Waiting for RPC connection...\n");
        rpc_connfd = accept(rpc_sockfd, NULL, NULL);
        //printf("Accepted RPC socket fd = %d\n", rpc_connfd);
        // TODO these 2 lines are specific
       size_t totalNumRead = 0;
        ssize_t numRead;
        bool error = false;
        while (totalNumRead < sizeof(lidar_cmd)) {
            if( (numRead = read(rpc_connfd, (char*)(&cmd)+totalNumRead, sizeof(lidar_cmd)-totalNumRead)) < 0) {
                std::cerr << "Lidar read error after " << totalNumRead << " bytes" << std::endl;
                error = true;
                break;
            }
            totalNumRead += numRead;
            if (numRead == 0) { //EOF
                break;
            }
        }
        if ((totalNumRead) < sizeof(lidar_cmd)||error)
        {
            std::cerr << "Invalid command reveived" << std::endl;
            std::cerr << "total read " << totalNumRead << " < of sizeof(lidar_cmd) expected " <<  sizeof(lidar_cmd) << std::endl;
            return;
        }

        // TODO fix later
        //if (cmd.rpc_socket_api_version != RPC_SOCKET_VERSION) {
        //    // TODO need to properly send back an error message in these
        //    break;
        //}

        // TODO these 2 lines are specific
        process(cmd.rawData, cmd.numPoints, cmd.horizontalAngle, cmd.channels); // call process using each element of cmd.data as args (i.e process(cmd.data[0], cmd.data[1].....))
        send(rpc_connfd, &cmd, sizeof(lidar_cmd), 0);

        if (close(rpc_connfd) == -1) {
            rpc_connfd = -1;
            std::cerr << "close" << std::endl;
            return;
        }
    }
}

void lidar_server::LIDAR_PROCESS
{
    try {
        char *error = NULL;
        for(auto &module: modules) {
            auto runFunc = (void (*)(float *, size_t, float&, size_t&))dlsym(module->handle, "run");
            if ((error = dlerror()) != NULL)  {
                printf("failed to find run func: %s\n", error);
            } else {
                runFunc(rawLidarPoints, numPoints, horizontalAngle, numChannels);
            }
        }
    } catch (std::exception &e) {
        std::cerr << "Could not process lidar pipeline:" <<std::endl;
        std::cerr << e.what() << std::endl;
    }
}

#ifdef WITH_MAIN
#include "../../python/include/lidar_module.hpp"
//#include <rclcpp/rclcpp.hpp>
int main(int argc, char** argv)
{
    //rclcpp::init(argc, argv);
    //auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
//
    //auto server = std::make_shared<lidar_server>();
    lidar_server server;
    auto t1 = server.runAPIThreaded();
    auto t2 = server.runRPCThreaded();
    t1.join();
    t2.join();
    //executor->add_node(server);
    //executor->spin();
    return 0;
}
#endif
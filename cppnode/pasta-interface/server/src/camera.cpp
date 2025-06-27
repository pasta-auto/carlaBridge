#include "../include/camera.hpp"
#include "../include/process_signatures.hpp"
#include "../include/remote_process.hpp"
#include <sys/socket.h>
#include <sys/un.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <algorithm>

camera_server::camera_server(std::string api, std::string rpc, std::string rnn): cmd_socket_server(api, rpc, rnn) {}

// TODO this is very copy paste is there some template stuff I can do where
//      camera_server can for example only implement<camera_cmd> problem different cmds, different params to process
void camera_server::runRPC()
{
    if (listen(rpc_sockfd, BACKLOG) == -1) {
        std::cerr << "listen" << std::endl;
        return;
    }

    camera_cmd cmd; // TODO specifc line

    for (;;) {
        //printf("Waiting for RPC connection...\n");
        rpc_connfd = accept(rpc_sockfd, NULL, NULL);
        //printf("Accepted RPC socket fd = %d\n", rpc_connfd);
        // TODO these 2 lines are specific
        // TODO need a time out here from what I see one call to read is not enough to get all data
        //      from say an image
        //while ((numRead = read(rpc_connfd, &cmd, sizeof(camera_cmd))) > 0) {
        // TODO do I have to deal with 2 images here? I think no cause right now RPC call is locking on send
        size_t totalNumRead = 0;
        ssize_t numRead;
        bool error = false;
        while (totalNumRead < sizeof(camera_cmd)) {
            if( (numRead = read(rpc_connfd, (char*)(&cmd)+totalNumRead, sizeof(camera_cmd)-totalNumRead)) < 0) {
                std::cerr << "Camera read error after " << totalNumRead << " bytes" << std::endl;
                error = true;
                break;
            }
            totalNumRead += numRead;
            if (numRead == 0) { //EOF
                break;
            }
        }
        if ((totalNumRead) < sizeof(camera_cmd)||error)
        {
            std::cerr << "Invalid command reveived" << std::endl;
            std::cerr << "total read " << totalNumRead << " < of sizeof(camera_cmd) expected " <<  sizeof(camera_cmd) << std::endl;
            return;
        }

        // TODO this doesn't work deal with it later
        //if (cmd.rpc_socket_api_version != RPC_SOCKET_VERSION) {
        //    // TODO need to properly send back an error message in these
        //    return;
        //}

        // TODO these 2 lines are specific
        process(cmd.rawData, cmd.width, cmd.height); 
        send(rpc_connfd, &cmd, sizeof(camera_cmd), 0);

        if (close(rpc_connfd) == -1) {
            rpc_connfd = -1;
            std::cerr << "close" << std::endl;
            return;
        }
    }
}

void camera_server::CAMERA_PROCESS
{
    try {
        char *error = NULL;
        for(auto &module: modules) {
            auto runFunc = (void (*)(uint8_t * rawData, size_t width, size_t height)) dlsym(module->handle, "run");
            
            if ((error = dlerror()) != NULL)  {
                printf("failed to find run func: %s\n", error);
            } else {
                runFunc(rawData, width, height);
            }
        }
    } catch (std::exception &e) {
        std::cerr << "Could not process camera pipeline:" <<std::endl;
        std::cerr << e.what() << std::endl;
    }
}

#ifdef WITH_MAIN
#include "../../python/include/camera_module.hpp"
//#include <rclcpp/rclcpp.hpp>
int main(int argc, char** argv)
{
    //rclcpp::init(argc, argv);
    //auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
//
    //auto server = std::make_shared<camera_server>();
    camera_server server;
    auto t1 = server.runAPIThreaded();
    auto t2 = server.runRPCThreaded();
    t1.join();
    t2.join();
    //executor->add_node(server);
    //executor->spin();
    return 0;
}
#endif
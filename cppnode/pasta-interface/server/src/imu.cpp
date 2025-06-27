#include "../include/imu.hpp"
#include "../include/remote_process.hpp"
#include <sys/socket.h>
#include <sys/un.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <algorithm>
imu_server::imu_server(std::string api, std::string rpc, std::string rnn): cmd_socket_server(api,rpc,rnn) {}

// TODO this is very copy paste is there some template stuff I can do where
//      imu_server can for example only implement<imu_cmd> problem different cmds, different params to process
void imu_server::runRPC()
{
    if (listen(rpc_sockfd, BACKLOG) == -1) {
        std::cerr << "listen" << std::endl;
        return;
    }

    ssize_t numRead;
    imu_cmd cmd; // TODO specifc line

    for (;;) {
        //printf("Waiting for RPC connection...\n");
        rpc_connfd = accept(rpc_sockfd, NULL, NULL);
        //printf("Accepted RPC socket fd = %d\n", rpc_connfd);
        // TODO these 2 lines are specific
        while ((numRead = read(rpc_connfd, &cmd, sizeof(imu_cmd))) > 0) {
            if (numRead < 0 || (size_t)(numRead) < sizeof(imu_cmd))
            {
                std::cerr << "Invalid command reveived" << std::endl;
                break;
            }

            if (cmd.rpc_socket_api_version != RPC_SOCKET_VERSION) {
                // TODO need to properly send back an error message in these
                break;
            }

            // TODO these lines are specific
            // TODO can't bind to packed field? so some struct thing quick googling says GCC specific thing when using __attribute(packed) going to ignore for now by awful copy paste
            //std::apply(&imu_server::process, cmd.data); // call process using each element of cmd.data as args (i.e process(cmd.data[0], cmd.data[1].....))
            float gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z, loc_x, loc_y, loc_z, yaw, pitch, roll;
            gyr_x = cmd.data[0]; gyr_y = cmd.data[ 1]; gyr_z = cmd.data[ 2];
            acc_x = cmd.data[3]; acc_y = cmd.data[ 4]; acc_z = cmd.data[ 5];
            loc_x = cmd.data[6]; loc_y = cmd.data[ 7]; loc_z = cmd.data[ 8];
            yaw   = cmd.data[9]; pitch = cmd.data[10];  roll = cmd.data[11];
            process(gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z, loc_x, loc_y, loc_z, yaw, pitch, roll);
            cmd.data[0] = gyr_x ; cmd.data[ 1] = gyr_y; cmd.data[ 2] = gyr_z;
            cmd.data[3] = acc_x ; cmd.data[ 4] = acc_y; cmd.data[ 5] = acc_z;
            cmd.data[6] = loc_x ; cmd.data[ 7] = loc_y; cmd.data[ 8] = loc_z;
            cmd.data[9] = yaw   ; cmd.data[10] = pitch; cmd.data[11] =  roll;

            send(rpc_connfd, &cmd, sizeof(imu_cmd), 0);

            if (close(rpc_connfd) == -1) {
                rpc_connfd = -1;
                std::cerr << "close" << std::endl;
                return;
            }
        }
    }
}

void imu_server::IMU_PROCESS
{
    float gyro[ ] = {gyr_x, gyr_y, gyr_z};
    float  acc[ ] = {acc_x, acc_y, acc_z};
    float  loc[ ] = {loc_x, loc_y, loc_z};
    float  rot[ ] = {  yaw, pitch,  roll};
    try {
        char *error = NULL;
        for(auto &module: modules) {
            auto runFunc = (void (*)(float *, float *, float *, float *))dlsym(module->handle, "run");
            if ((error = dlerror()) != NULL)  {
                printf("failed to find run func: %s\n", error);
            } else {
                runFunc(&gyro[0], &acc[0], &loc[0], &rot[0]);
            }
        }
        gyr_x = gyro[0]; gyr_y = gyro[1]; gyr_z = gyro[2];
        acc_x =  acc[0]; acc_y =  acc[1]; acc_z =  acc[2];
        loc_x =  loc[0]; loc_y =  loc[1]; loc_z =  loc[2];
          yaw =  rot[0]; pitch =  rot[1];  roll =  rot[2];
    } catch (std::exception &e) {
        std::cerr << "Could not process imu pipeline:" <<std::endl;
        std::cerr << e.what() << std::endl;
    }
}

#ifdef WITH_MAIN
#include "../../python/include/imu_module.hpp"
//#include <rclcpp/rclcpp.hpp>
int main(int argc, char** argv)
{
    //rclcpp::init(argc, argv);
    //auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
//
    //auto server = std::make_shared<imu_server>();
    imu_server server;
    auto t1 = server.runAPIThreaded();
    auto t2 = server.runRPCThreaded();
    t1.join();
    t2.join();
    //executor->add_node(server);
    //executor->spin();
    return 0;
}
#endif
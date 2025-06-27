#pragma once
#include "server.hpp"
#include "process_signatures.hpp"

class imu_server : public cmd_socket_server
{
public:
    imu_server(std::string api_socket_path = IMU_API_SOCKET, std::string rpc_socket_path = IMU_RPC_SOCKET, std::string ros_node_name = "IMUAPIServer");
    void IMU_PROCESS;
private:
    void runRPC() override;
};
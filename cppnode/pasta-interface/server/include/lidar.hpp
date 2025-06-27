#pragma once
#include "server.hpp"
#include "process_signatures.hpp"

class lidar_server : public cmd_socket_server
{
public:
    lidar_server(std::string api_socket_path = LIDAR_API_SOCKET, std::string rpc_socket_path = LIDAR_RPC_SOCKET, std::string ros_node_name = "LiDARAPIServer");
    void LIDAR_PROCESS;
private:
    void runRPC() override;
};
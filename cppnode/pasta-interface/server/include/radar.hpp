#pragma once
#include "server.hpp"
#include "process_signatures.hpp"

class radar_server : public cmd_socket_server
{
public:
    radar_server(std::string api_socket_path = RADAR_API_SOCKET, std::string rpc_socket_path = RADAR_RPC_SOCKET, std::string ros_node_name = "RadarAPIServer");
    void RADAR_PROCESS;
private:
    void runRPC() override;
};
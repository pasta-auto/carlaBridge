#pragma once
#include "server.hpp"
#include "process_signatures.hpp"

class camera_server : public cmd_socket_server
{
public:
    camera_server(std::string api_socket_path = CAMERA_API_SOCKET, std::string rpc_socket_path = CAMERA_RPC_SOCKET, std::string ros_node_name = "CameraAPIServer");
    void CAMERA_PROCESS;
private:
    void runRPC() override;
};
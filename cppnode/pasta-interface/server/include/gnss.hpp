#pragma once
#include "server.hpp"
#include "process_signatures.hpp"

class gnss_server : public cmd_socket_server
{
public:
    gnss_server(std::string api_socket_path = GNSS_API_SOCKET, std::string rpc_socket_path = GNSS_RPC_SOCKET, std::string ros_node_name = "GNSSAPIServer");
    // this is kind of awkward think need to pass by address here rather than reference because I want to make a ndarray
    // so somehting like double* args[lat,long,alt]; ndarry = numpy::from_data(args);
    void GNSS_PROCESS;
private:
    void runRPC() override;
};
#pragma once
#include "server.hpp"
#include "process_signatures.hpp"

#include <queue>
#include <tuple>

class can_server : public cmd_socket_server
{
public:
    can_server(std::string api_socket_path = CAN_API_SOCKET, std::string rpc_socket_path = CAN_RPC_SOCKET);
    // this is kind of awkward think need to pass by address here rather than reference because I want to make a ndarray
    // so somehting like double* args[lat,long,alt]; ndarry = numpy::from_data(args);
    void CAN_PROCESS;
    void CAN_SEND;
private:
    void runRPC() override;
    std::queue<std::tuple<uint16_t, uint64_t, int>> can_data_to_send;
};
#include "../include/can.hpp"
#include "../include/remote_process.hpp"
#include <sys/socket.h>
#include <sys/un.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <algorithm>
#include <set>
#include <tuple>
can_server::can_server(std::string api, std::string rpc): cmd_socket_server(api,rpc) {}


// TODO this is very copy paste is there some template stuff I can do where
//      can_server can for example only implement<can_cmd> problem different cmds, different params to process
void can_server::runRPC()
{
    if (listen(rpc_sockfd, BACKLOG) == -1) {
        std::cerr << "listen" << std::endl;
        return;
    }

    ssize_t numRead;
    CAN_CMD_UNION cmd;

    for (;;) {
        //printf("Waiting for RPC connection...\n");
        rpc_connfd = accept(rpc_sockfd, NULL, NULL);
        //printf("Accepted RPC socket fd = %d\n", rpc_connfd);
        // TODO these 2 lines are specific

        while ((numRead = read(rpc_connfd, &cmd, sizeof(CAN_CMD_UNION))) > 0) {
            if (numRead < 0 || (size_t)(numRead) < sizeof(RPM_can_cmd_base) || cmd.base.can_cmd_id == CAN_CMD_INVALID)
            {
                std::cerr << "Invalid command reveived" << std::endl;
                break;
            }

            if (cmd.base.rpc_socket_api_version != RPC_SOCKET_VERSION) {
                // TODO need to properly send back an error message in these
                break;
            }

            if (cmd.base.can_cmd_id == CAN_CMD_PROCESS)
            {
                uint64_t id = cmd.process.data[0], data = cmd.process.data[1], size = cmd.process.data[2];
                process(id, data, size);

                cmd.process.data[0] = id; cmd.process.data[1] = data; cmd.process.data[2] = size;
                send(rpc_connfd, &cmd.process, sizeof(can_cmd), 0);
            } else if (cmd.base.can_cmd_id == CAN_CMD_SEND_REQ) {
                std::queue<std::tuple<uint16_t, uint64_t, int>> data;

                process_send(data);
                cmd.send_resp.can_size = data.size();

                int i = 0;
                while(!data.empty())  
                {
                    auto msg = data.front();
                    data.pop();
                    cmd.send_resp.data[i][0] = std::get<0>(msg);
                    cmd.send_resp.data[i][1] = std::get<1>(msg);
                    cmd.send_resp.data[i][2] = std::get<2>(msg);

                    i++;

                    if (i >= CAN_MAX_SIZE)
                    {
                        break;
                    }
                }          

                cmd.send_resp.can_size = i;   
                send(rpc_connfd, &cmd.send_resp, sizeof(cmd.send_resp), 0); 
            } else {
                std::cerr << "Invalid command reveived" << std::endl;
            };

            if (close(rpc_connfd) == -1) {
                rpc_connfd = -1;
                std::cerr << "close" << std::endl;
                return;
            }
        }
    }
}

void can_server::CAN_PROCESS
{
    uint64_t args[ ] = { id, data, size} ;
    try {
        char *error = NULL;
        for(auto &module: modules) {
            std::set<int> id_filter;
            auto filter_get = (std::set<int> (*)())dlsym(module->handle, "filter_get");
            if ((error = dlerror()) != NULL)  {
                printf("failed to find filter_get func: %s\n", error);
                continue;
            } else {
                id_filter = filter_get();
            }

            if (!id_filter.empty() && id_filter.count(id) == 0)
            {
                continue;
            }

            auto runFunc = (void (*)(uint64_t *))dlsym(module->handle, "run");
            if ((error = dlerror()) != NULL)  {
                printf("failed to find run func: %s\n", error);
            } else {
                runFunc(&args[0]);
            }
        }
    } catch (std::exception &e) {
        std::cerr << "Could not process can pipeline:" <<std::endl;
        std::cerr << e.what() << std::endl;
    }
}

void can_server::CAN_SEND
{
    try {
        char *error = NULL;
        for(auto &module: modules) {
            std::queue<std::tuple<uint16_t, uint64_t, int>> send_queue;
            auto send_get = (std::queue<std::tuple<uint16_t, uint64_t, int>> (*)(int))dlsym(module->handle, "send_get");

            if ((error = dlerror()) != NULL)  {
                printf("failed to find run func: %s\n", error);
                continue;
            } else {
                int max_get = CAN_MAX_SIZE - data.size();
                if (max_get == 0)
                {
                    continue;
                }

                send_queue = send_get(max_get);
            }

            while(!send_queue.empty())
            {
                data.push(send_queue.front());
                send_queue.pop();
            }
        }
    } catch (std::exception &e) {
        std::cerr << "Could not process can send pipeline:" <<std::endl;
        std::cerr << e.what() << std::endl;
    }
}

#ifdef WITH_MAIN
#include "../../python/include/can_module.hpp"
int main(int argc, char** argv)
{
    auto server = std::make_shared<can_server>();

    auto t1 = server->runAPIThreaded();
    auto t2 = server->runRPCThreaded();

    t1.join();
    t2.join();
    return 0;
}
#endif
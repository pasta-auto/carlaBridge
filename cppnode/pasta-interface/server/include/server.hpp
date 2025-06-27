#pragma once

#include <string>
#include <iostream>
#include <vector>

//#include <rclcpp/rclcpp.hpp>

#if __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#elif __has_include(<filesystem>)
#include <filesystem>
using fs = std::filesystem;
#endif

#include <dlfcn.h>

// #include <format>
#include "../../include/commands.hpp"
#include "../../include/pasta_interface.hpp"
// #define errMsg(msg, ...) (std::cerr << std::format(msg, ...) << std::cout)

#include <thread>

#define DLOPEN_TYPE void*

#define BACKLOG 5

struct module_data
{
    DLOPEN_TYPE handle;
    fs::path path;
    module_info* info;
};


class cmd_socket_server //: public rclcpp::Node
{
public:
    cmd_socket_server(std::string api_socket_path, std::string rpc_socket_path = "", std::string ros_node_name = "Server");
    void runAPI();
    virtual void runRPC() { }; // does nothing in base needs a process function
    std::thread runAPIThreaded();
    std::thread runRPCThreaded();

protected:
    std::vector<module_data*> modules;
    int rpc_sockfd; // protected because derive classes will use it
    int rpc_connfd;

private:
    void cmd_version_mismatch(cmd_base* cmd);
    void cmd_unknown_response(cmd_base* cmd);
    void cmd_version_response(cmd_base* cmd);
    void cmd_add_response(cmd_base* cmd);
    void cmd_remove_response(cmd_base* cmd);
    void cmd_list_response(cmd_base* cmd);
    void cmd_reload_response(cmd_base* cmd);
    void cmd_show_response(cmd_base* cmd);
    void cmd_move_response(cmd_base* cmd);
    void cmd_swap_response(cmd_base* cmd);

    std::string api_socket_path;
    std::string rpc_socket_path;

    bool boundsCheck(size_t idx);

    void send_response(bool error=false, const std::string & errorMsg="", bool verbose = false, size_t verboseIdx=-1);

    int api_sockfd;
    int api_connfd;

    
};
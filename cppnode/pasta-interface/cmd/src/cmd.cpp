#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <variant>
#include <iostream>
#include <sstream>
#include <boost/algorithm/string.hpp>

#if __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#elif __has_include(<filesystem>)
#include <filesystem>
using fs = std::filesystem;
#endif

#include "../include/cmd.hpp"
#include "../../server/include/process_signatures.hpp"

#define ADD_SERVER_OPTION                                                            \
    add_option("Server_Type", server, "Which server to send to")->required()->check( \
        CLI::IsMember({                                                              \
            "can", "camera", "gnss", "imu", "lidar", "radar"                         \
        }, CLI::ignore_case, CLI::ignore_underscore)                                 \
    );

#define ADD_SERVER_OPTION_NO_REQ                                         \
    add_option("Server_Type", server, "Which server to send to")->check( \
        CLI::IsMember({                                                  \
            "can", "camera", "gnss", "imu", "lidar", "radar"             \
        }, CLI::ignore_case, CLI::ignore_underscore)                     \
    );


const std::map<std::string, std::string> nameToSocketMap = {
    {"can"   ,   CAN_API_SOCKET},
    {"camera",CAMERA_API_SOCKET},
    {"gnss"  ,  GNSS_API_SOCKET},
    {"imu"   ,   IMU_API_SOCKET},
    {"lidar" , LIDAR_API_SOCKET},
    {"radar" , RADAR_API_SOCKET}
};

class sockRAII{
    public:
        int sfd;
        sockRAII() {this->sfd = socket(AF_UNIX, SOCK_STREAM, 0);}
        ~sockRAII() {close(this->sfd);}
};

int handleResp(int sfd, bool versionCmd = false)
{
    ssize_t numRead;
    char buf[sizeof(response_error)];
    //Wait for command response
    //TODO Timeout on response
    //TODO sizeof should be w/e the size of the largest response?
    while ((numRead = read(sfd, buf, sizeof(response_error))) <= 0);

    response_base* response = reinterpret_cast<response_base*>(buf);
    
    if (check_response_error(response))
    {
        //do something for error
    } else if (versionCmd)
    {
       response_version* resp_typed = reinterpret_cast<response_version*>(response);
       std::cout << "Server Version: v" << resp_typed->socket_api_version / 100 << "." << resp_typed->socket_api_version % 100 << std::endl;
       std::cout << "Client Version: v" << API_SOCKET_VERSION / 100 << "." << API_SOCKET_VERSION % 100 << std::endl;
    }
    std::cout << "Loaded modules:" << std::endl;
    std::cout << response->loaded_modules;

    if (numRead == -1) {
      std::cerr << "Read error" << std::endl;
        return -1;
    }
    return EXIT_SUCCESS;
}

void list_for_server(struct sockaddr_un& addr, bool verbose)
{
    sockRAII sock;
    if (connect(sock.sfd, (struct sockaddr *) &addr, sizeof(struct sockaddr_un)) == -1) {
        std::cerr << "Could not connect to server" << std::endl;
        return;
    }
    cmd_list cmd;
    cmd.verbose = verbose;
    bool succ = socket_write(cmd, sock.sfd);
    if (!succ) {
        std::cerr << "Could not send to server" << std::endl;
        return;
    }
    handleResp(sock.sfd);
}

int main(int argc, char* argv[])
{
    using vtype=std::variant<int, double, std::string>;
    vtype v1, v2;
    bool verbose = false;
    size_t idx1, idx2; //TODO just using v1 getting error if pass variant as guess defaulting to string
    std::string server = "";

    CLI::App app("PASTA API Module Control");
    CLI::App &version = *app.add_subcommand("version", "Get server and client versions");

    CLI::App &add = *app.add_subcommand("add", "Add PASTA API module");
    add.ADD_SERVER_OPTION;
    add.add_option("file", v1, "Module SO file to load")->check(CLI::ExistingFile)->required();

    CLI::App &remove = *app.add_subcommand("remove", "Remove PASTA API module");
    remove.ADD_SERVER_OPTION;
    remove.add_option("file", v1, "Module SO to remove")->required(); // file not ExistingFile required so can remove from server if source so is gone

    CLI::App &list = *app.add_subcommand("list", "List the PASTA API modules");
    list.ADD_SERVER_OPTION_NO_REQ;
    list.add_flag("-v,--verbose", verbose, "Fully describe each module rather than just a module list");

    CLI::App &reload = *app.add_subcommand("reload", "Reload a module given its index");
    reload.ADD_SERVER_OPTION;
    reload.add_option("index", idx1, "Index to reload")->required();

    CLI::App &show = *app.add_subcommand("show", "Show full information about a module given its index");
    show.ADD_SERVER_OPTION;
    show.add_option("index", idx1, "Index to show information")->required();

    CLI::App &move = *app.add_subcommand("move", "Will alter the module execution order by moving the module at a given index to a given position.");
    move.ADD_SERVER_OPTION;
    move.add_option("index"   , idx1, "Index to move")      ->required();
    move.add_option("position", idx2, "Position to move to")->required();

    CLI::App &swap = *app.add_subcommand("swap", "Will alter the module execution order by swaping two modules given their indices");
    swap.ADD_SERVER_OPTION;
    swap.add_option("index_1", idx1, "Index of the first module to move") ->required();
    swap.add_option("index_2", idx2, "Index of the second module to move")->required();
    
    app.require_subcommand(1);
    // I don't think there is a way to put this into one option / validator
    //std::stringstream s;
    //std::function<std::string(std::string &)> validatorFunc = [&list, &s](std::string & inputType){
    //    // if sub command is list and a server is provided or if it is not list
    //    // make sure server type is valid
    //    if(((list && inputType != "") || !list) && (
    //           boost::iequals(inputType, "can")
    //        || boost::iequals(inputType, "camera")
    //        || boost::iequals(inputType, "gnss")
    //        || boost::iequals(inputType, "imu")
    //        || boost::iequals(inputType, "lidar")
    //        || boost::iequals(inputType, "radar")
    //    )) {
    //        return std::string("");
    //    } else {
    //        s << "Error server type is not valid; server must be one of: [";
    //        for (const auto& [key, val] : nameToSocketMap) s << key << " ";
    //        s << "]" << std::endl;
    //        s << bool(list) << std::endl;
    //        s << (list && inputType != "") << std::endl;
    //        return s.str();
    //    }
    //};

    CLI11_PARSE(app, argc, argv);

    //Setup what subcommand it was executed
    uint8_t sub_cmd = 0;

    if (version)
    {
        sub_cmd = CMD_SOCKET_VERSION;
    } else if (add)
    {
        sub_cmd = CMD_ADD;
    } else if (remove)
    {
        sub_cmd = CMD_REMOVE;
    } else if (list)
    {
        sub_cmd = CMD_LIST;
    } else if(reload)
    {
        sub_cmd = CMD_RELOAD;
    } else if (show)
    {
        sub_cmd = CMD_SHOW;
    } else if (move)
    {
        sub_cmd = CMD_MOVE;
    } else if (swap)
    {
        sub_cmd = CMD_SWAP;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(struct sockaddr_un));
    addr.sun_family = AF_UNIX;

    // Connects the active socket referred to be sfd to the listening socket
    // whose address is specified by addr.
    int sfd;
    if (sub_cmd != CMD_LIST) {
        // Create a new client socket with domain: AF_UNIX, type: SOCK_STREAM, protocol: 0
        sfd = socket(AF_UNIX, SOCK_STREAM, 0);

        // Make sure socket's file descriptor is legit.
        if (sfd == -1) {
            std::cerr << "socket" << std::endl;
            return -1;
        }
        strncpy(addr.sun_path, nameToSocketMap.at(server).c_str(), sizeof(addr.sun_path) - 1);
        if (connect(sfd, (struct sockaddr *) &addr,
                    sizeof(struct sockaddr_un)) == -1) {
        std::cerr << "connect" << std::endl;
        return -1;
        }
    }

    bool write_suc = false;
    switch(sub_cmd)
    {
        case CMD_SOCKET_VERSION:
        {
            cmd_version cmd;
            write_suc = socket_write(cmd, sfd);
            break;
        }
        case CMD_ADD:
        {
            cmd_add cmd;
            fs::path path = std::get<std::string>(v1);
            strcpy(cmd.path, fs::absolute(path).c_str());
            write_suc = socket_write(cmd, sfd);
            break;
        }
        case CMD_REMOVE:
        {
            cmd_remove cmd;
            fs::path path = std::get<std::string>(v1);
            strcpy(cmd.path, fs::absolute(path).c_str());
            write_suc = socket_write(cmd, sfd);
            break;
        }
        case CMD_LIST:
        {
            if (server != "") { // server specified only list for one server
                strncpy(addr.sun_path, nameToSocketMap.at(server).c_str(), sizeof(addr.sun_path) - 1);
                list_for_server(addr, verbose);
            } else { // get all servers
                for (const auto& [key, val] : nameToSocketMap) {
                    std::cout << "----------- List of modules for " << key << " server -----------" << std::endl;
                    strncpy(addr.sun_path, nameToSocketMap.at(key).c_str(), sizeof(addr.sun_path) - 1);
                    list_for_server(addr, verbose);
                }
                write_suc = true;
            }
            
            break;
        }
        case CMD_RELOAD:
        {
            cmd_reload cmd;
            cmd.idx = idx1;
            write_suc = socket_write(cmd, sfd);
            break;
        }
        case CMD_SHOW:
        {
            cmd_show cmd;
            cmd.idx = idx1;
            write_suc = socket_write(cmd, sfd);
            break;
        }
        case CMD_MOVE:
        {
            cmd_move cmd;
            cmd.idx = idx1;
            cmd.pos = idx2;
            write_suc = socket_write(cmd, sfd);
            break;
        }
        case CMD_SWAP:
        {
            cmd_swap cmd;
            cmd.idx1 = idx1;
            cmd.idx2 = idx2;
            write_suc = socket_write(cmd, sfd);
            break;
        }
    }

    if (!write_suc)
    {
        //TODO Error messages
        return -1;
    }

    if (sub_cmd != CMD_LIST) {
        return handleResp(sfd, sub_cmd == CMD_SOCKET_VERSION);
    }

    // Closes our socket; server sees EOF.
    return EXIT_SUCCESS;
}

bool check_response_error(response_base* resp)
{
    if (resp->error)
    {
        std::cerr << reinterpret_cast<response_error*>(resp)->error_msg << std::endl;
        return true;
    } 

    return false;
}

template<class CMD_TYPE> bool socket_write(CMD_TYPE cmd, int sfd)
{
    if (write(sfd, &cmd, sizeof(cmd)) != sizeof(cmd)) {
        std::cerr << "partial/failed write" << std::endl;
        return false;
    }

    return true;
}
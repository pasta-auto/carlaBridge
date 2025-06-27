#include "../include/server.hpp"

#include <sys/socket.h>
#include <sys/un.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <algorithm>

#define BUF_SIZE 300

bool bindSocket(int& sockfd, const std::string& path)
{
    struct sockaddr_un addr;
    
    // Create a new server socket with domain: AF_UNIX, type: SOCK_STREAM, protocol: 0
    sockfd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sockfd == -1) { // Make sure socket's file descriptor is legit.
        std::cerr << "socket" << std::endl;
        return false;
    }
    if (strlen(path.c_str()) > sizeof(addr.sun_path) - 1) {
        std::cerr << "Server socket path too long: " << path << std::endl;
        return false;
    }
    // Delete any file that already exists at the address. Make sure the deletion
    // succeeds. If the error is just that the file/directory doesn't exist, it's fine.
    if (remove(path.c_str()) == -1 && errno != ENOENT) {
        std::cerr << "remove-" << path << std::endl;
        return false;
    }
    // Zero out the address, and set family and path.
    memset(&addr, 0, sizeof(struct sockaddr_un));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);

    // Bind the socket to the address. Note that we're binding the server socket
    // to a well-known address so that clients know where to connect.
    if (bind(sockfd, (struct sockaddr *) &addr, sizeof(struct sockaddr_un)) == -1) {
        std::cerr << "bind" << std::endl;
        return false;
    }

    return true;
}

cmd_socket_server::cmd_socket_server(std::string apiPath, std::string rpcPath, std::string ros_node_name) //: Node(ros_node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
    api_socket_path = apiPath;
    rpc_socket_path = rpcPath;

    if (!bindSocket(api_sockfd, api_socket_path)) {
        std::cerr << "Failed to bind api socket to " << api_socket_path << std::endl;
        return;
    }
    printf("Server API socket fd = %d\n", api_sockfd);

    if (rpc_socket_path.length() > 0) { // this defaults to "" so if provided a path bind RPC socket as well
        if (!bindSocket(rpc_sockfd, rpc_socket_path)) {
            std::cerr << "Failed to bind rpc socket to " << rpc_socket_path << std::endl;
            return;
        }
        printf("Server RPC socket fd = %d\n", rpc_sockfd);
    }
}

std::thread cmd_socket_server::runAPIThreaded() {
    return std::thread([this] {runAPI();} );
}

std::thread cmd_socket_server::runRPCThreaded() {
    return std::thread([this] {runRPC();});
}

void cmd_socket_server::runAPI()
{
    // The listen call marks the socket as *passive*. The socket will subsequently
    // be used to accept connections from *active* sockets.
    // listen cannot be called on a connected socket (a socket on which a connect()
    // has been succesfully performed or a socket returned by a call to accept()).
    if (listen(api_sockfd, BACKLOG) == -1) {
        std::cerr << "listen" << std::endl;
        return;
    }

    ssize_t numRead;
    char buf[BUF_SIZE];
    for (;;) {          /* Handle client connections iteratively */

        // Accept a connection. The connection is returned on a NEW
        // socket, 'api_connfd'; the listening socket ('api_sockfd') remains open
        // and can be used to accept further connections. */
        printf("Waiting to accept an API connection...\n");
        // NOTE: blocks until a connection request arrives.
        api_connfd = accept(api_sockfd, NULL, NULL);
        //printf("Accepted socket fd = %d\n", api_connfd);

        while ((numRead = read(api_connfd, buf, BUF_SIZE)) > 0) {
            if (numRead < 0 || (size_t)(numRead) < sizeof(cmd_base))
            {
                std::cerr << "Invalid command reveived" << std::endl;
                break;
            } 

            cmd_base* cmd = (cmd_base*)buf;

            //Server check
            if (cmd->socket_api_version != API_SOCKET_VERSION)
            {
                cmd_version_mismatch(cmd);
                break;
            }

            switch (cmd->cmd)
            {
                case CMD_SOCKET_VERSION:
                cmd_version_response(cmd);
                break;
                case CMD_ADD:
                cmd_add_response(cmd);
                break;
                case CMD_REMOVE:
                cmd_remove_response(cmd);
                break;
                case CMD_LIST:
                cmd_list_response(cmd);
                break;
                case CMD_RELOAD:
                cmd_reload_response(cmd);
                break;
                case CMD_SHOW:
                cmd_show_response(cmd);
                break;
                case CMD_MOVE:
                cmd_move_response(cmd);
                break;
                case CMD_SWAP:
                cmd_swap_response(cmd);
                break;
                default:
                cmd_unknown_response(cmd);
                break;
                break;
            };
        }

        if (numRead == -1) {
            std::cerr << "read" << std::endl;
            return;
        }

        if (close(api_connfd) == -1) {
            api_connfd = -1;
            std::cerr << "close" << std::endl;
            return;
        }
    }
}

void cmd_socket_server::send_response(bool error, const std::string & errorMsg, bool verbose, size_t verboseIdx)
{
    response_error response;
    response.error = error;
    if (error) {
        snprintf(response.error_msg, ERROR_MSG_BUFF_SIZE, "%s", errorMsg.c_str());
    }
    std::stringstream s;
    // set the underlying buffer for string stream to what we want the output list to end up in
    s.rdbuf()->pubsetbuf(response.loaded_modules, MOD_LIST_BUFF_SIZE);
    for(size_t i = 0; i<modules.size(); i++)
    {
        auto module = modules[i];
        s << std::to_string(i) << ": " << module->info->name << " (" << module->path.string() << ")" << std::endl;
        if (verbose || i == verboseIdx) {
            s << *module->info;
        }
    }
    // need to signal we are done writing to loaded module buffer
    s << '\0';
    response.loaded_modules[MOD_LIST_BUFF_SIZE-1] = '\0'; // just in case
    std::cout << "Currently loaded modules:" << std::endl;
    std::cout << response.loaded_modules; // NOTE: don't use s.str() gets funky when change underlying buf
    send(api_connfd, &response, sizeof(response), 0);
}

void cmd_socket_server::cmd_version_mismatch(cmd_base* cmd)
{
    char buff[ERROR_MSG_BUFF_SIZE];
    snprintf(buff, ERROR_MSG_BUFF_SIZE,  "Error: Version Mismatch\n\tServer Version: %i\n\tClient Version: %i", cmd->socket_api_version, API_SOCKET_VERSION);
    send_response(true, buff);
}

void cmd_socket_server::cmd_unknown_response(cmd_base* cmd)
{
    char buff[ERROR_MSG_BUFF_SIZE];
    snprintf(buff, ERROR_MSG_BUFF_SIZE,  "Error: Unknown server command %i", cmd->cmd);
    send_response(true, buff);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void cmd_socket_server::cmd_version_response(cmd_base* cmd)
{
    send_response();
}
#pragma GCC diagnostic pop

void cmd_socket_server::cmd_add_response(cmd_base* cmd)
{
    cmd_add* add = reinterpret_cast<cmd_add*>(cmd);
    // this makes the .so unique either do this or change the remove to have all option or w/e
    auto modComp =  [&add](auto module){
        return module->path == add->path;
    };
    if (std::find_if(modules.begin(), modules.end(), modComp) != modules.end()) {
        send_response(true, "Error: Module already in loaded");
        return;
    }
    module_data* data = new module_data;
    data->path = add->path;
    //data->handle = dlmopen(LM_ID_NEWLM, add->path, RTLD_NOW);
    data->handle = dlopen(add->path, RTLD_NOW|RTLD_LOCAL);
    if (!data->handle) {
        std::cout << "Could not open: " << add->path << std::endl;
        std::cout << dlerror() << std::endl;
        send_response(true, "Error: Failed to open with that path");
        delete(data);
        return;
    }

    data->info = reinterpret_cast<module_info*>(dlsym(data->handle, "info"));
    if (!data->info) {
        std::cout << "No module info in module: " << add->path << std::endl;
        send_response(true, "Error: Invalid module; no module data found");
        delete(data);
        return;
    }

    auto module_init = (void (*)())dlsym(data->handle, "module_init");
    if (module_init)  {
        module_init();
    } 

    modules.push_back(data);
    send_response();
}

void cmd_socket_server::cmd_list_response(cmd_base* cmd)
{
    response_version response;

    cmd_list* list = reinterpret_cast<cmd_list*>(cmd); 
    send_response(false, "", list->verbose);
}

void cmd_socket_server::cmd_remove_response(cmd_base* cmd)
{
    cmd_remove* remove = reinterpret_cast<cmd_remove*>(cmd);

    std::cout << std::string(remove->path) << std::endl;
    auto modComp =  [&remove](auto module){
        return module->path == remove->path;
    };
    if (auto modToRemove = std::find_if(modules.begin(), modules.end(), modComp); modToRemove != modules.end()) {
        dlclose((*modToRemove)->handle);
        delete(*modToRemove);
        modules.erase(modToRemove);
        send_response();
    } else {
        send_response(true, "Error: No module with that path");
    }
}

// adding this as think will have this all over the place so just gives short form
bool cmd_socket_server::boundsCheck(size_t idx) {
    if (modules.size() == 0 || idx > modules.size() - 1) {
        send_response(true, "Index out of bounds");
        return true;
    }
    return false;
}
void cmd_socket_server::cmd_reload_response(cmd_base* cmd){
    cmd_reload* reload = reinterpret_cast<cmd_reload*>(cmd);
    if(boundsCheck(reload->idx)) {return;}
    auto module = modules[reload->idx];
    
    dlclose(module->handle);    
    //module->handle = dlmopen(LM_ID_NEWLM, module->path.string().c_str(), RTLD_NOW);
    module->handle = dlopen(module->path.string().c_str(), RTLD_NOW|RTLD_LOCAL);
    if (!module->handle) {
        std::cout << "Could not open: " << module->path << std::endl;
        send_response(true, "Error: Failed to reopen when reloading removing from list");
        modules.erase(modules.begin() + reload->idx);
        delete(module);
        return;
    }
    module->info = reinterpret_cast<module_info*>(dlsym(module->handle, "info"));
    if (!module->info) {
        std::cout << "No module info in module: " << module->path << std::endl;
        send_response(true, "Error: Invalid module found on reloading");
        modules.erase(modules.begin() + reload->idx);
        delete(module);
        return;
    }

    auto module_init = (void (*)())dlsym(module->handle, "module_init");
    if (module_init)  {
        module_init();
    } 

    send_response();;
}
void cmd_socket_server::cmd_show_response(cmd_base* cmd) {
    cmd_show* show = reinterpret_cast<cmd_show*>(cmd);
    if(boundsCheck(show->idx)) {return;}
    send_response(false, "", false, show->idx);
}
void cmd_socket_server::cmd_move_response(cmd_base* cmd) {
    cmd_move* move = reinterpret_cast<cmd_move*>(cmd);
    if(boundsCheck(move->idx) || boundsCheck(move->pos)) {return;}
    std::cout << "Move request " << std::to_string(move->idx) << " -> " << std::to_string(move->pos) << std::endl;
    if (move->idx > move->pos)
        std::rotate(modules.rend() - move->idx - 1, modules.rend() - move->idx, modules.rend() - move->pos);
    else        
        std::rotate(modules.begin() + move->idx, modules.begin() + move->idx + 1, modules.begin() + move->pos + 1);
    send_response();
}
void cmd_socket_server::cmd_swap_response(cmd_base* cmd) {
    cmd_swap* swap = reinterpret_cast<cmd_swap*>(cmd);
    if(boundsCheck(swap->idx1) || boundsCheck(swap->idx2)) {return;}
    std::cout << "Swap request " << std::to_string(swap->idx1) << " <--> " << std::to_string(swap->idx2) << std::endl;
    std::iter_swap(modules.begin() + swap->idx1, modules.begin() + swap->idx2);
    send_response();
}
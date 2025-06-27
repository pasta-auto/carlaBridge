#pragma once

#define API_SOCKET_VERSION 1
#define RPC_SOCKET_VERSION 1

constexpr size_t PATH_BUFF_SIZE = 255;
constexpr size_t ERROR_MSG_BUFF_SIZE = 100;
// this could get pretty big pretty quickly this might not be enough 
constexpr size_t MOD_LIST_BUFF_SIZE = 1024;

enum Module_Types {
    MODULE_TYPE_TEST = 255
};

enum Cmds {
    CMD_SOCKET_VERSION = 1,
    CMD_ADD,
    CMD_REMOVE,
    CMD_LIST,
    CMD_RELOAD,
    CMD_SHOW,
    CMD_MOVE,
    CMD_SWAP
};

struct cmd_base {
    uint8_t socket_api_version = API_SOCKET_VERSION;
    uint8_t cmd;
} __attribute__((packed));

struct cmd_version : cmd_base{
    cmd_version() { cmd = CMD_SOCKET_VERSION; }
} __attribute__((packed));

struct cmd_list : cmd_base{
    cmd_list() { cmd = CMD_LIST; }
    bool verbose;
} __attribute__((packed));

struct cmd_add : cmd_base{
    cmd_add() { cmd = CMD_ADD; }
    char path[PATH_BUFF_SIZE];
} __attribute__((packed));

struct cmd_remove : cmd_base{
    cmd_remove() { cmd = CMD_REMOVE; }
    char path[PATH_BUFF_SIZE];
} __attribute__((packed));

struct cmd_reload : cmd_base{
    cmd_reload() { cmd = CMD_RELOAD; }
    size_t idx;
} __attribute__((packed));

struct cmd_show : cmd_base{
    cmd_show() { cmd = CMD_SHOW; }
    size_t idx;
} __attribute__((packed));

struct cmd_move : cmd_base{
    cmd_move() { cmd = CMD_MOVE; }
    size_t idx; size_t pos;
} __attribute__((packed));

struct cmd_swap : cmd_base{
    cmd_swap() { cmd = CMD_SWAP; }
    size_t idx1; size_t idx2;
} __attribute__((packed));

struct response_base {
    uint8_t socket_api_version = API_SOCKET_VERSION;
    bool error = false;
    char loaded_modules[MOD_LIST_BUFF_SIZE];
} __attribute__((packed));

struct response_version : response_base {} __attribute__((packed));

struct response_error : response_base
{
    char error_msg[ERROR_MSG_BUFF_SIZE];
} __attribute__((packed));


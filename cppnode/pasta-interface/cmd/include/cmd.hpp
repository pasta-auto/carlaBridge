#pragma once

#include "../../include/commands.hpp"
#include "../include/CLI11.hpp"


bool check_response_error(response_base* resp);

template<class CMD_TYPE> bool socket_write(CMD_TYPE cmd, int sfd);

//CMD's
// void cmd_version();
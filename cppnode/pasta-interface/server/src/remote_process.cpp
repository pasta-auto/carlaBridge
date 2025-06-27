#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include "../include/remote_process.hpp"

class sockRAII{
    public:
        int sfd;
        sockRAII() {this->sfd = socket(AF_UNIX, SOCK_STREAM, 0);}
        ~sockRAII() {close(this->sfd);}
};

template<typename CMD_TYPE, typename CMD_TYPE_RESP = CMD_TYPE> bool shared_RPC_process(const struct sockaddr_un * addr, CMD_TYPE * cmd, CMD_TYPE_RESP * cmd_resp = nullptr)
{
    if (cmd_resp == nullptr)
    {
        cmd_resp = reinterpret_cast<CMD_TYPE_RESP*>(cmd);
    }

    sockRAII sock;
    // Make sure socket's file descriptor is legit.
    if (sock.sfd == -1) {
        std::cerr << "Failed to initialize socket" << std::endl;
        return false;
    }
    if (connect(sock.sfd, (struct sockaddr *)addr, sizeof(struct sockaddr_un)) == -1)
    {
        std::cerr << "Failed to connect to: " << addr->sun_path << std::endl;
        return false;
    }

    ssize_t writeSize = write(sock.sfd, cmd, sizeof(CMD_TYPE));
    if (writeSize != sizeof(CMD_TYPE)) {
        std::cerr << "partial/failed write" << std::endl;
        return false;
    }
    size_t totalNumRead = 0;
    ssize_t numRead = 0;;

    while (totalNumRead < sizeof(CMD_TYPE_RESP)) {
        if( (numRead = read(sock.sfd, (char*)(cmd_resp)+totalNumRead, sizeof(CMD_TYPE_RESP)-totalNumRead)) < 0) {
            std::cerr << "Response read error after " << totalNumRead << " bytes" << std::endl;
            return false;
        }
        totalNumRead += numRead;
        if (numRead == 0) { //EOF
            break;
        }
    }
    if (totalNumRead < sizeof(CMD_TYPE_RESP)) {
        std::cerr << "Response smaller than expected: " << totalNumRead << "/" << sizeof(CMD_TYPE_RESP) << std::endl;
        return false;
    }

    return true;
}

remote_process::remote_process()
{
    auto addrInit = [](struct sockaddr_un * addr, const std::string& path){
        memset(addr, 0, sizeof(struct sockaddr_un));
        addr->sun_family = AF_UNIX;
        strncpy(addr->sun_path, path.c_str(), sizeof(addr->sun_path) - 1);
    };
    addrInit(&camera_sock_addr,CAMERA_RPC_SOCKET);
    addrInit(&  gnss_sock_addr,  GNSS_RPC_SOCKET);
    addrInit(&   imu_sock_addr,   IMU_RPC_SOCKET);
    addrInit(& lidar_sock_addr, LIDAR_RPC_SOCKET);
    addrInit(& radar_sock_addr, RADAR_RPC_SOCKET);
    addrInit(&   can_sock_addr,   CAN_RPC_SOCKET);
}

bool remote_process::CONCATENATE(camera_,CAMERA_PROCESS)
{
    std::lock_guard lk(camera_mutex);
    camera_cmd cmd;
    cmd.height = height;
    cmd.width  = width;
    memcpy(&cmd.rawData, rawData, 4*width*height*sizeof(uint8_t));

    if(!shared_RPC_process<camera_cmd>(&camera_sock_addr, &cmd)) {
        std::cerr << "Camera process failed" << std::endl;
        return false;
    }
    
    memcpy(rawData, &cmd.rawData, 4*width*height*sizeof(uint8_t));
    return true;
}


bool remote_process::CONCATENATE(gnss_, GNSS_PROCESS)
{
    std::lock_guard lk(gnss_mutex);

    gnss_cmd cmd;
    cmd.data[0] = latitude;
    cmd.data[1] = longitude;
    cmd.data[2] = altitude;
    
    if(!shared_RPC_process(&gnss_sock_addr, &cmd)) {
        std::cerr << "GNSS process failed" << std::endl;
        return false;
    }

    latitude  = cmd.data[0];
    longitude = cmd.data[1];
    altitude  = cmd.data[2];
    return true;
}

bool remote_process::CONCATENATE(imu_, IMU_PROCESS)
{
    std::lock_guard lk(imu_mutex);

    imu_cmd cmd;
    cmd.data[0] = gyr_x ; cmd.data[ 1] = gyr_y; cmd.data[ 2] = gyr_z;
    cmd.data[3] = acc_x ; cmd.data[ 4] = acc_y; cmd.data[ 5] = acc_z;
    cmd.data[6] = loc_x ; cmd.data[ 7] = loc_y; cmd.data[ 8] = loc_z;
    cmd.data[9] = yaw   ; cmd.data[10] = pitch; cmd.data[11] =  roll;
   
    if(!shared_RPC_process(&imu_sock_addr, &cmd)) {
        std::cerr << "IMU process failed" << std::endl;
        return false;
    }

    gyr_x = cmd.data[0]; gyr_y = cmd.data[ 1]; gyr_z = cmd.data[ 2];
    acc_x = cmd.data[3]; acc_y = cmd.data[ 4]; acc_z = cmd.data[ 5];
    loc_x = cmd.data[6]; loc_y = cmd.data[ 7]; loc_z = cmd.data[ 8];
    yaw   = cmd.data[9]; pitch = cmd.data[10];  roll = cmd.data[11];
    return true;
}

bool remote_process::CONCATENATE(lidar_, LIDAR_PROCESS)
{
    std::lock_guard lk(lidar_mutex);

    lidar_cmd cmd;
    cmd.numPoints       = numPoints;
    cmd.horizontalAngle = horizontalAngle;
    cmd.channels        = numChannels;
    memcpy(&cmd.rawData, rawLidarPoints, numPoints*4*sizeof(float));

    if(!shared_RPC_process(&lidar_sock_addr, &cmd)) {
        std::cerr << "Lidar process failed" << std::endl;
        return false;
    }
    
    memcpy(rawLidarPoints, &cmd.rawData, numPoints*4*sizeof(float));
    return true;
}

bool remote_process::CONCATENATE(radar_, RADAR_PROCESS)
{
    std::lock_guard lk(radar_mutex);

    radar_cmd cmd;
    cmd.numPoints = numPoints;

    memcpy(&cmd.rawData, rawData, numPoints*4*sizeof(float));

    if(!shared_RPC_process(&radar_sock_addr, &cmd)) {
        std::cerr << "Radar process failed" << std::endl;
        return false;
    }
    
    memcpy(rawData, &cmd.rawData, numPoints*4*sizeof(float));
    return true;
}

bool remote_process::CONCATENATE(can_, CAN_PROCESS)
{
    std::lock_guard lk(can_mutex);

    can_cmd cmd;
    cmd.data[0] = id;
    cmd.data[1] = data;
    cmd.data[2] = size;

    if(!shared_RPC_process(&can_sock_addr, &cmd)) {
        std::cerr << "Can process failed" << std::endl;
        return false;
    }

    return true;
}

bool remote_process::CONCATENATE(can_, CAN_SEND)
{
    std::lock_guard lk(can_mutex);

    can_send_req_cmd cmd;
    can_send_resp_cmd cmd_resp;

    if(!shared_RPC_process(&can_sock_addr, &cmd, &cmd_resp)) {
        std::cerr << "Can process failed" << std::endl;
        return false;
    }

    for (int i = 0; i < cmd_resp.can_size; i++)
    {
        int id = cmd_resp.data[i][0];
        int can_data = cmd_resp.data[i][1];
        int size = cmd_resp.data[i][2];

        data.push(std::make_tuple(id, can_data, size));
    }

    return true;
}
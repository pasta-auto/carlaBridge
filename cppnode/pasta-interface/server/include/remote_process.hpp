#pragma once
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <sys/un.h>
#include "process_signatures.hpp"
#include "commands.hpp"


// TODO should I just send a size first than raw bytes like camera_cmd send width height than just make buffer size exact?
constexpr size_t CAM_MAX_SIZE = 640*480*4; // arbitrary but assigning on stack currently so if to big have to malloc it
constexpr size_t LIDAR_MAX_SIZE = 100000*4; // arbitrary
constexpr size_t RADAR_MAX_SIZE = 1000*4; // arbitrary
constexpr size_t CAN_MAX_SIZE = 25; // arbitrary

enum CAN_CMD_COMMANDS {
    CAN_CMD_INVALID = 0,
    CAN_CMD_PROCESS,
    CAN_CMD_SEND_REQ,
    CAN_CMD_SEND_RESP
};

struct RPC_cmd_base {
    uint8_t rpc_socket_api_version = RPC_SOCKET_VERSION;
};

struct camera_cmd : RPC_cmd_base {
    size_t width, height;
    uint8_t rawData[CAM_MAX_SIZE];
} __attribute__((packed));

struct gnss_cmd: RPC_cmd_base {
    double data[3]; // lat, lon, alt
} __attribute__((packed));

struct RPM_can_cmd_base: RPC_cmd_base {
    RPM_can_cmd_base() {}
    uint8_t can_cmd_id = 0;
} __attribute__((packed));

struct can_cmd: RPM_can_cmd_base {
    can_cmd() { can_cmd_id = CAN_CMD_PROCESS; }
    uint64_t data[3]; // id, data, size
} __attribute__((packed));

struct can_send_req_cmd: RPM_can_cmd_base {
    can_send_req_cmd() { can_cmd_id = CAN_CMD_SEND_REQ; }
} __attribute__((packed));

struct can_send_resp_cmd: RPM_can_cmd_base {
    can_send_resp_cmd() { can_cmd_id = CAN_CMD_SEND_RESP; }
    size_t can_size;
    uint64_t data[CAN_MAX_SIZE][3]; // id, data, size
} __attribute__((packed));

union CAN_CMD_UNION
{
    RPM_can_cmd_base base;
    can_cmd process;
    can_send_req_cmd send_req;
    can_send_resp_cmd send_resp;
    CAN_CMD_UNION() {};
    ~CAN_CMD_UNION() {};
};

struct imu_cmd: RPC_cmd_base {
    float data[12]; // 3xgyro, 3xacc, 3xloc, 3xrot
} __attribute__((packed));

struct lidar_cmd: RPC_cmd_base {
    size_t numPoints; size_t channels;
    float horizontalAngle;
    float rawData[LIDAR_MAX_SIZE];
};// __attribute__((packed)); intentionally not packed as want to have a cmd buffer that read/write from and the packed may mess with that
// not a problem in cmaera with uint8_t buffer but is here and in radar

struct radar_cmd: RPC_cmd_base {
    size_t numPoints;
    float rawData[RADAR_MAX_SIZE];
};// __attribute__((packed)); intentionally not packed see above

template<typename CMD_TYPE> bool shared_RPC_process(int socketfd, const struct sockaddr_un * addr, CMD_TYPE * cmd);

class remote_process {
    public:
        remote_process();
        bool CONCATENATE(camera_, CAMERA_PROCESS);
        bool CONCATENATE(  gnss_,   GNSS_PROCESS);
        bool CONCATENATE(   imu_,    IMU_PROCESS);
        bool CONCATENATE( lidar_,  LIDAR_PROCESS);
        bool CONCATENATE( radar_,  RADAR_PROCESS);
        bool CONCATENATE(   can_,    CAN_PROCESS);
        bool CONCATENATE(   can_,    CAN_SEND);
    private:
    // socket names
        struct sockaddr_un camera_sock_addr;
        struct sockaddr_un   gnss_sock_addr;
        struct sockaddr_un    imu_sock_addr;
        struct sockaddr_un  lidar_sock_addr;
        struct sockaddr_un  radar_sock_addr;
        struct sockaddr_un    can_sock_addr;
    // mutexes can run each of these concurrently buy not planning on letting same one run
        std::mutex camera_mutex;
        std::mutex   gnss_mutex;
        std::mutex    imu_mutex;
        std::mutex  lidar_mutex;
        std::mutex  radar_mutex;
        std::mutex    can_mutex;
};
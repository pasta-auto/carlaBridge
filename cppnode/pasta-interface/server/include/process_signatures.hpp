#pragma once
#include <string>
#include <queue>
#include <tuple>

// default socket names
const std::string CAMERA_API_SOCKET("/tmp/camera.socket");
const std::string   GNSS_API_SOCKET(  "/tmp/gnss.socket");
const std::string    IMU_API_SOCKET(   "/tmp/imu.socket");
const std::string  LIDAR_API_SOCKET( "/tmp/lidar.socket");
const std::string  RADAR_API_SOCKET( "/tmp/radar.socket");
const std::string    CAN_API_SOCKET(   "/tmp/can.socket");
const std::string CAMERA_RPC_SOCKET("/tmp/camera.socket.data");
const std::string   GNSS_RPC_SOCKET(  "/tmp/gnss.socket.data");
const std::string    IMU_RPC_SOCKET(   "/tmp/imu.socket.data");
const std::string  LIDAR_RPC_SOCKET( "/tmp/lidar.socket.data");
const std::string  RADAR_RPC_SOCKET( "/tmp/radar.socket.data");
const std::string    CAN_RPC_SOCKET(   "/tmp/can.socket.data");

// TODO this is pretty ugly but basic gist is going to make for example camera_server::process do the SO loading stuff
//      and then make remote_connection_to_server::camera_process and I want the exact same function signature between the two
// TODO is there a point to doing this if not also going to do it in cpp file too? leaving for now just to extra stess if change this change 
//        camera.hpp etc and remote_process.hpp
#define CAMERA_PROCESS process(uint8_t * const rawData, size_t width, size_t height)
#define   GNSS_PROCESS process(double &latitude, double &longitude, double &altitude)
#define    IMU_PROCESS process( \
                           float & gyr_x, float & gyr_y, float & gyr_z \
                         , float & acc_x, float & acc_y, float & acc_z \
                         , float & loc_x, float & loc_y, float & loc_z \
                         , float & yaw  , float & pitch, float & roll  \
                       )
#define  RADAR_PROCESS process(float * rawData, size_t numPoints)
#define  LIDAR_PROCESS process(float * rawLidarPoints, size_t numPoints, float horizontalAngle, size_t numChannels)

#define  CAN_PROCESS process(uint16_t id, uint64_t data, int size)
#define  CAN_SEND process_send(std::queue<std::tuple<uint16_t, uint64_t, int>> &data)

#define CONCATENATE_DETAIL(x, y) x##y
#define CONCATENATE(x, y) CONCATENATE_DETAIL(x, y)
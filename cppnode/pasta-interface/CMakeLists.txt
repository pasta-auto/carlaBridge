cmake_minimum_required(VERSION 3.8)
project(pasta-interface)

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


FIND_PACKAGE(Python3 COMPONENTS Interpreter Development)
FIND_PACKAGE(PythonInterp)

if (PYTHONINTERP_FOUND)
  FIND_PACKAGE(Boost COMPONENTS python numpy)
  FIND_PACKAGE(PythonInterp 3)
  FIND_PACKAGE(PythonLibs 3 REQUIRED)
else()
    message("Python not found")
endif()

message(STATUS "Python3::Interpreter = ${Python3_Interpreter}")
message(STATUS "PYTHON_LIBRARIES = ${PYTHON_LIBRARIES}")
message(STATUS "PYTHON_EXECUTABLE = ${PYTHON_EXECUTABLE}")
message(STATUS "PYTHON_INCLUDE_DIRS = ${PYTHON_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}")
message(STATUS "Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}")

add_executable(test_server
    server/include/server.hpp
    server/src/server.cpp
    example/testServer.cpp
)
ament_target_dependencies(test_server
  "rclcpp"
)

add_executable(cmd_interface
    cmd/src/cmd.cpp
    cmd/include/CLI11.hpp
)
target_link_libraries(cmd_interface PUBLIC stdc++fs)
target_compile_features(cmd_interface PUBLIC cxx_std_17)
install(TARGETS cmd_interface)

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} include server python/include)

# this would be repeated for each API type
# TODO might need to muck with python path after this
#   to do w/e this function am replacing does: Python3_add_library(test_module STATIC python/src/python_test_module.cpp )
add_library(test_module STATIC python/src/test_module.cpp)
set_property(TARGET test_module PROPERTY POSITION_INDEPENDENT_CODE ON)
SET_TARGET_PROPERTIES(test_module PROPERTIES PREFIX "")
target_link_libraries(test_module ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
install(TARGETS test_module LIBRARY DESTINATION lib)

add_library(camera_module STATIC python/src/camera_module.cpp)
set_property(TARGET camera_module PROPERTY POSITION_INDEPENDENT_CODE ON)
SET_TARGET_PROPERTIES(camera_module PROPERTIES PREFIX "")
install(TARGETS camera_module LIBRARY DESTINATION lib)

add_library(gnss_module STATIC python/src/gnss_module.cpp)
set_property(TARGET gnss_module PROPERTY POSITION_INDEPENDENT_CODE ON)
SET_TARGET_PROPERTIES(gnss_module PROPERTIES PREFIX "")
install(TARGETS gnss_module LIBRARY DESTINATION lib)

add_library(lidar_module STATIC python/src/lidar_module.cpp)
set_property(TARGET lidar_module PROPERTY POSITION_INDEPENDENT_CODE ON)
SET_TARGET_PROPERTIES(lidar_module PROPERTIES PREFIX "")
install(TARGETS lidar_module LIBRARY DESTINATION lib)

add_library(imu_module STATIC python/src/imu_module.cpp)
set_property(TARGET imu_module PROPERTY POSITION_INDEPENDENT_CODE ON)
SET_TARGET_PROPERTIES(imu_module PROPERTIES PREFIX "")
install(TARGETS imu_module LIBRARY DESTINATION lib)

add_library(radar_module STATIC python/src/radar_module.cpp)
set_property(TARGET radar_module PROPERTY POSITION_INDEPENDENT_CODE ON)
SET_TARGET_PROPERTIES(radar_module PROPERTIES PREFIX "")
install(TARGETS radar_module LIBRARY DESTINATION lib)

add_library(can_module STATIC python/src/can_module.cpp)
set_property(TARGET can_module PROPERTY POSITION_INDEPENDENT_CODE ON)
SET_TARGET_PROPERTIES(can_module PROPERTIES PREFIX "")
install(TARGETS can_module LIBRARY DESTINATION lib)

target_link_libraries(cmd_interface PUBLIC stdc++fs)
target_link_libraries(test_server stdc++fs)

target_compile_features(cmd_interface PUBLIC cxx_std_17)
target_compile_features(test_server PUBLIC cxx_std_17)

add_executable(
    camera_server
    server/include/server.hpp
    server/src/server.cpp
    server/include/camera.hpp
    server/src/camera.cpp
)
target_compile_features(camera_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(camera_server PUBLIC WITH_MAIN)
target_link_libraries(camera_server stdc++fs camera_module ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
ament_target_dependencies(camera_server
  "rclcpp"
)
install(TARGETS camera_server)

add_executable(
    gnss_server
    server/include/server.hpp
    server/src/server.cpp
    server/include/gnss.hpp
    server/src/gnss.cpp
)
target_compile_features(gnss_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(gnss_server PUBLIC WITH_MAIN)
target_link_libraries(gnss_server stdc++fs ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} gnss_module)
ament_target_dependencies(gnss_server
  "rclcpp"
)
install(TARGETS gnss_server)

add_executable(
    imu_server
    server/include/server.hpp
    server/src/server.cpp
    server/include/imu.hpp
    server/src/imu.cpp
)
target_compile_features(imu_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(imu_server PUBLIC WITH_MAIN)
target_link_libraries(imu_server stdc++fs imu_module ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
ament_target_dependencies(imu_server
  "rclcpp"
)
install(TARGETS imu_server)

add_executable(
    radar_server
    server/include/server.hpp
    server/src/server.cpp
    server/include/radar.hpp
    server/src/radar.cpp
)
target_compile_features(radar_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(radar_server PUBLIC WITH_MAIN)
target_link_libraries(radar_server stdc++fs radar_module ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
ament_target_dependencies(radar_server
  "rclcpp"
)
install(TARGETS radar_server)

add_executable(
    lidar_server
    server/include/server.hpp
    server/src/server.cpp
    server/include/lidar.hpp
    server/src/lidar.cpp
)
target_compile_features(lidar_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(lidar_server PUBLIC WITH_MAIN)
target_link_libraries(lidar_server stdc++fs lidar_module ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
ament_target_dependencies(lidar_server
  "rclcpp"
)
install(TARGETS lidar_server)

add_executable(
    can_server
    server/include/server.hpp
    server/src/server.cpp
    server/include/can.hpp
    server/src/can.cpp
)
target_compile_features(can_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(can_server PUBLIC WITH_MAIN)
target_link_libraries(can_server stdc++fs can_module ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
install(TARGETS can_server)

add_library(
    remote_process STATIC
    server/src/remote_process.cpp
    server/include/remote_process.hpp
)

#ament_export_include_directories(remote_process server/include)
install(TARGETS remote_process LIBRARY DESTINATION lib)


install(
  DIRECTORY server/include/
  DESTINATION include/${PROJECT_NAME}/server
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_libraries(remote_process)
ament_export_include_directories("include/${PROJECT_NAME}")

#ADD_SUBDIRECTORY(example)
ament_package()


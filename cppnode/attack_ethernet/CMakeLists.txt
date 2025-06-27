cmake_minimum_required(VERSION 3.8)
project(attack_ethernet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(attack_ethernet_msgs REQUIRED)
find_package(autoware_sensing_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(autoware_auto_mapping_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(gnss_poser REQUIRED)
find_package(tier4_map_msgs REQUIRED)
find_package(geo_pos_conv REQUIRED)

## Find non-ROS library
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)

set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

include_directories(include/${PROJECT_NAME})
ament_export_include_directories("include/${PROJECT_NAME}")

add_executable(${PROJECT_NAME} 
  src/attack_ethernet.cpp
  src/config.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "builtin_interfaces"
  "sensor_msgs"
  "std_msgs"
  "attack_ethernet_msgs"
  "autoware_sensing_msgs"
  "autoware_auto_vehicle_msgs"
  "autoware_auto_control_msgs"
  "tier4_vehicle_msgs"
  "autoware_adapi_v1_msgs"
  "geo_pos_conv"
)

target_link_libraries(${PROJECT_NAME}
  Geographic
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()

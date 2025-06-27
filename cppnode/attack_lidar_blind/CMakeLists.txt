cmake_minimum_required(VERSION 3.8)
project(attack_lidar_blind)

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
find_package(attack_lidar_blind_msgs REQUIRED)
find_package(autoware_auto_mapping_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_extension REQUIRED)

include_directories(include/${PROJECT_NAME})
ament_export_include_directories("include/${PROJECT_NAME}")

add_executable(${PROJECT_NAME} 
  src/attack_lidar_blind.cpp
  src/config.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "builtin_interfaces"
  "sensor_msgs"
  "autoware_auto_mapping_msgs"
  "std_msgs"
  "geometry_msgs"
  "lanelet2_core"
  "lanelet2_extension"
  "attack_lidar_blind_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_eigen"
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()

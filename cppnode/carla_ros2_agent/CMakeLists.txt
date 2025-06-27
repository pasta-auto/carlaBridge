cmake_minimum_required(VERSION 3.8)
project(carla_ros2_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include("/root/git/carla/LibCarla/cmake/CMakeLists.txt")
# list(APPEND CMAKE_MODULE_PATH "/root/git/carla/LibCarla/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_sensing_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(tier4_external_api_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(fmt REQUIRED)
find_package(dummy_perception_publisher REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(pasta-interface REQUIRED)

## Find non-ROS library
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)

set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

include(carla_client)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include/${PROJECT_NAME})
ament_export_include_directories("include/${PROJECT_NAME}")

add_executable(carla_ros2_agent 
  src/agent.cpp
  src/autoware_ros2_agent.cpp
  src/carla_ros2_agent.cpp
  src/carla_ros2_bridge.cpp
  src/world_handler.cpp
  src/agent_handler.cpp
  src/carla_ego_spawn_handler.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "builtin_interfaces"
  "sensor_msgs"
  "std_msgs"
  "autoware_sensing_msgs"
  "autoware_auto_vehicle_msgs"
  "autoware_auto_control_msgs"
  "tier4_vehicle_msgs"
  "autoware_adapi_v1_msgs"
  "tier4_external_api_msgs"
  "ackermann_msgs"
  "tf2"
  "fmt"
  "dummy_perception_publisher"
  "autoware_auto_perception_msgs"
  "pasta-interface"
)

# TODO does this mean this gets built twice if I made it a package?
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../pasta-interface pasta-interface)
# TODO I really borked this thing trying to eliminate above
# find_package(pasta-interface REQUIRED)
target_link_libraries(carla_ros2_agent
  carla_client
  fmt::fmt-header-only
  Geographic
)

target_include_directories(carla_ros2_agent PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(carla_ros2_agent PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS carla_ros2_agent
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

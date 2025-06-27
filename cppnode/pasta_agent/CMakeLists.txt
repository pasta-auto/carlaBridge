cmake_minimum_required(VERSION 3.8)
project(pasta_agent)

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
find_package(tier4_vehicle_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(tier4_external_api_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(fmt REQUIRED)
find_package(pasta-interface REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include/${PROJECT_NAME})
ament_export_include_directories("include/${PROJECT_NAME}")

add_executable(pasta_agent 
  src/pasta_agent.cpp
  src/ros2_agent.cpp
  src/pasta_interface.cpp
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
  "ackermann_msgs"
  "autoware_adapi_v1_msgs"
  "tier4_external_api_msgs"
  "tf2"
  "tf2_ros"
  "fmt"
  "pasta-interface"
)

target_link_libraries(pasta_agent
  fmt::fmt-header-only
)

target_include_directories(pasta_agent PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pasta_agent PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS pasta_agent
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
cmake_minimum_required(VERSION 3.8)
project(attack_gnss_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GNSSAttackStatus.msg"
  "msg/GNSSAttackConfig.msg"
  DEPENDENCIES std_msgs
 )

rosidl_get_typesupport_target(cpp_typesupport_target  
${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)

ament_package()

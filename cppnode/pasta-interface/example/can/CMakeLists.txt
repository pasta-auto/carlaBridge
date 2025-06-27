cmake_minimum_required(VERSION 3.8)

project(can_test)

find_package(ament_cmake REQUIRED)
find_package(pasta-interface REQUIRED)


ADD_EXECUTABLE(client client.cpp)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} -fno-stack-protector" )
TARGET_LINK_LIBRARIES(client ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} remote_process)

ament_target_dependencies(client
  "pasta-interface"
)
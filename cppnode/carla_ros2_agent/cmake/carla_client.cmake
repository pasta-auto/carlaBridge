add_library(carla_rpc STATIC IMPORTED )
set_target_properties(carla_rpc PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/libs/carla_build/lib/librpc.a")

add_library(carla_recast STATIC IMPORTED )
set_target_properties(carla_recast PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/libs/carla_build/lib/libRecast.a")

add_library(carla_detour STATIC IMPORTED )
set_target_properties(carla_detour PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/libs/carla_build/lib/libDetour.a")

add_library(carla_detour_crowd STATIC IMPORTED )
set_target_properties(carla_detour_crowd PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/libs/carla_build/lib/libDetourCrowd.a")

add_library(carla_boost STATIC IMPORTED )
set_target_properties(carla_boost PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/libs/carla_build/lib/libboost_filesystem.a")

add_library(carla_client STATIC IMPORTED)
set_target_properties(carla_client PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/libs/carla_build/lib/libcarla_client.a")
set_target_properties(carla_client PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/libs/carla_build/include;${CMAKE_CURRENT_SOURCE_DIR}/libs/carla_build/include/system")

set_property(TARGET carla_client PROPERTY IMPORTED_LINK_INTERFACE_LIBRARIES carla_rpc carla_detour carla_detour_crowd carla_recast carla_boost)

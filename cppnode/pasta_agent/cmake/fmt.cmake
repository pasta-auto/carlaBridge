add_library(fmt STATIC )
target_include_directories(fmt PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/libs/fmt/include" )
target_sources(fmt PRIVATE
"${CMAKE_CURRENT_SOURCE_DIR}/libs/fmt/src/format.cc"
)

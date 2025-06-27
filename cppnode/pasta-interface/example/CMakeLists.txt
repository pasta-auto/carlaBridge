# this should be always run and will build all py files with matching yaml files in a directory so not just exPyModule
# TODO figure out how to properly run this basically need to only run it if a yaml file changes
# TODO instead of above maybe better to do the check in the generate script? so something like:
#      for each name.yaml in the --target dir check if name.so exists in the target dir
#      if so exists and it was last modified more recently than the .yaml skip generation for that yaml file
# TODO this next line is bit of a hack but raiotnale is expect to call this CMake from autoware so may not have the example folder so force it to be created
add_custom_target(build-time-make-directory ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory  ${CMAKE_BINARY_DIR}/example)
add_custom_target(
    exPyModule.so
    ALL
    COMMAND ${CMAKE_CURRENT_LIST_DIR}/../python/stubGeneration/generateStub.py --target_dir ${CMAKE_CURRENT_LIST_DIR}/../example -o ${CMAKE_BINARY_DIR}/example
)
#TODO removing everything else from being built for use in docker file as only maybe want examples but more importantly required module .SOs
#INCLUDE_DIRECTORIES(../include ../python/include)
#
## NOTE: this is weird naming combination
## this is test_module.cpp is supposed to be a user implemented module implemented with our APIs
#add_library(a SHARED
#    test_module.cpp
#)
#add_library(b SHARED
#    test_module.cpp
#)
#add_library(c SHARED
#    test_module.cpp
#)
#SET_TARGET_PROPERTIES(a PROPERTIES PREFIX "")
#SET_TARGET_PROPERTIES(b PROPERTIES PREFIX "")
#SET_TARGET_PROPERTIES(c PROPERTIES PREFIX "")
## here where I am linking test_module I am not refering to the above but as a type of API
## as in expect test_module gnss_module can_data_module etc. (TODO should this be renamed to API?)
#ADD_EXECUTABLE(testPyPipeline testPythonPipeline_noServer.cpp)
#TARGET_LINK_LIBRARIES(testPyPipeline ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} test_module camera_module)
#
#ADD_EXECUTABLE(minNumpyEx minNumpyLock.cpp)
#TARGET_LINK_LIBRARIES(minNumpyEx ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} test_module camera_module)

ADD_EXECUTABLE(readSO readSo.cpp)
TARGET_LINK_LIBRARIES(readSO ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})


#ADD_SUBDIRECTORY(carla)
#ADD_SUBDIRECTORY(can)
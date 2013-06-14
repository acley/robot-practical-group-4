cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_boost_directories()
rosbuild_add_executable(pickup_action_server src/PickupActionServer.cpp)
rosbuild_add_executable(kick_action_server src/KickActionServer.cpp)
rosbuild_add_executable(walk_action_server src/WalkActionServer.cpp)
rosbuild_add_executable(drop_action_server src/DropActionServer.cpp)
rosbuild_add_executable(walkholding_action_server src/WalkHoldingActionServer.cpp)
rosbuild_add_executable(turn_action_server src/TurnActionServer.cpp)
rosbuild_link_boost(pickup_action_server thread)
rosbuild_link_boost(drop_action_server thread)
rosbuild_link_boost(kick_action_server thread)
rosbuild_link_boost(walk_action_server thread)
rosbuild_link_boost(walkholding_action_server thread)
rosbuild_link_boost(turn_action_server thread)

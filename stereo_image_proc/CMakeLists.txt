cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Release)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_boost_directories()

# Dynamic reconfigure support
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# Processing library and node
rosbuild_add_library(stereo_image_proc src/libstereo_image_proc/processor.cpp)

rosbuild_add_executable(stereoimageproc_exe src/nodes/stereo_image_proc.cpp)
target_link_libraries(stereoimageproc_exe stereo_image_proc)
SET_TARGET_PROPERTIES(stereoimageproc_exe PROPERTIES OUTPUT_NAME stereo_image_proc)

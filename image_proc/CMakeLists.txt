cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rospack(image_proc)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
rospack_add_boost_directories()

rospack_add_library(imageproc src/proc/image.cpp)

rospack_add_executable(image_proc src/nodes/image_proc.cpp)
target_link_libraries(image_proc imageproc)

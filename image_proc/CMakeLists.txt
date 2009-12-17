cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
rosbuild_add_boost_directories()

rosbuild_add_library(imageproc src/proc/image.cpp)

rosbuild_add_executable(image_proc src/nodes/image_proc.cpp)
target_link_libraries(image_proc imageproc)

rosbuild_add_executable(image_proc_rostest test/rostest.cpp)
rosbuild_add_gtest_build_flags(image_proc_rostest)

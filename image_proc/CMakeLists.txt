cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
rosbuild_add_boost_directories()

# Dynamic reconfigure support
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# Nodelet library
rosbuild_add_library(image_proc src/libimage_proc/processor.cpp src/nodelets/debayer.cpp src/nodelets/rectify.cpp src/libimage_proc/advertisement_checker.cpp)

# Standalone node
rosbuild_add_executable(image_proc_exe src/nodes/image_proc.cpp)
target_link_libraries(image_proc_exe image_proc)
SET_TARGET_PROPERTIES(image_proc_exe PROPERTIES OUTPUT_NAME image_proc)

# Tests
rosbuild_add_executable(image_proc_rostest test/rostest.cpp)
rosbuild_add_gtest_build_flags(image_proc_rostest)

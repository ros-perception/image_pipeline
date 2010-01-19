cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

# Image viewers
rosbuild_add_executable(image_view image_view.cpp)

rosbuild_add_executable(stereo_view stereo_view.cpp)

# Extra tools (unstable)
rosbuild_add_executable(extract_images extract_images.cpp)

rosbuild_add_executable(image_saver image_saver.cpp)

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Image viewers
rosbuild_add_executable(image_view image_view.cpp)

rosbuild_add_executable(stereo_view stereo_view.cpp)

# Extra tools (unstable)
rosbuild_add_executable(extract_images extract_images.cpp)

rosbuild_add_executable(image_saver image_saver.cpp)

# Hack to remove binaries previously built in the package root.
execute_process(COMMAND cmake -E remove ${PROJECT_SOURCE_DIR}/image_view ${PROJECT_SOURCE_DIR}/stereo_view ${PROJECT_SOURCE_DIR}/extract_images ${PROJECT_SOURCE_DIR}/image_saver)

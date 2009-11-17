cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Release)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_boost_directories()

# for dynamic parameters, yippee!
# need to have config file in cfg/xx.cfg
#  and need to set its executable bit
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# this will go away when the new stereo_image_proc node becomes the default
rosbuild_add_library(stereoproc src/proc/stereoimage.cpp src/proc/stereolib.c src/proc/stereolib2.cpp)
rosbuild_add_compile_flags(stereoproc "-msse2 -mpreferred-stack-boundary=4")

rosbuild_add_executable(stereoproc_exe src/nodes/stereoproc.cpp)
target_link_libraries(stereoproc_exe stereoproc)
SET_TARGET_PROPERTIES(stereoproc_exe PROPERTIES OUTPUT_NAME stereoproc)

rosbuild_add_executable(stereoimageproc_exe src/nodes/stereo_image_proc.cpp)
target_link_libraries(stereoimageproc_exe stereoproc)
SET_TARGET_PROPERTIES(stereoimageproc_exe PROPERTIES OUTPUT_NAME stereo_image_proc)

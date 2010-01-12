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

# New processing library and node
#rosbuild_add_library(stereo_image_proc src/libstereo_image_proc/processor.cpp src/proc/stereolib.c src/proc/stereolib2.cpp src/proc/stereoimage.cpp)
rosbuild_add_library(stereo_image_proc src/libstereo_image_proc/processor.cpp src/proc/stereolib.c)
rosbuild_add_compile_flags(stereo_image_proc "-msse2 -mpreferred-stack-boundary=4")

rosbuild_add_executable(stereoimageproc_exe src/nodes/stereo_image_proc.cpp)
target_link_libraries(stereoimageproc_exe stereo_image_proc)
SET_TARGET_PROPERTIES(stereoimageproc_exe PROPERTIES OUTPUT_NAME stereo_image_proc)

# DEBUG old version for comparison
#rosbuild_add_executable(stereo_image_proc_old src/nodes/stereo_image_proc_stable.cpp)
#target_link_libraries(stereo_image_proc_old stereo_image_proc)

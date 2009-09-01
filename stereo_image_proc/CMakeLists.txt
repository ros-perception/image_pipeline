cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Release)

rospack(stereo_image_proc)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_boost_directories()

rospack_add_library(stereoproc src/proc/stereoimage.cpp src/proc/stereolib.c src/proc/stereolib2.cpp)
rospack_add_compile_flags(stereoproc "-msse2 -mpreferred-stack-boundary=4")

rospack_add_executable(stereoproc_exe src/nodes/stereoproc.cpp)
target_link_libraries(stereoproc_exe stereoproc imageproc)
SET_TARGET_PROPERTIES(stereoproc_exe PROPERTIES OUTPUT_NAME stereoproc)

rospack_add_executable(stereoimageproc_exe src/nodes/stereo_image_proc.cpp)
target_link_libraries(stereoimageproc_exe stereoproc imageproc)
SET_TARGET_PROPERTIES(stereoimageproc_exe PROPERTIES OUTPUT_NAME stereo_image_proc)

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Release)

rospack(stereo_image_proc)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_boost_directories()

rospack_add_library(stereoproc src/image.cpp src/stereolib.c src/stereolib2.cpp src/findplane.cpp)
rospack_add_compile_flags(stereoproc "-msse2 -mpreferred-stack-boundary=4")

rospack_add_executable(stereoproc_exe src/stereoproc.cpp)
rospack_add_compile_flags(stereoproc_exe "-msse2 -mpreferred-stack-boundary=4")
target_link_libraries(stereoproc_exe stereoproc)
SET_TARGET_PROPERTIES(stereoproc_exe PROPERTIES OUTPUT_NAME stereoproc)

rospack_add_executable(imageproc src/imageproc.cpp)
rospack_add_compile_flags(imageproc "-msse2 -mpreferred-stack-boundary=4")
target_link_libraries(imageproc stereoproc)

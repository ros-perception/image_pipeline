cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(image_view)

rospack_add_executable(image_view image_view.cpp)
rospack_add_executable(image_view_tmp image_view_tmp.cpp)

rospack_add_executable(stereo_view stereo_view.cpp)

rospack_add_executable(test_stereo test_stereo.cpp)

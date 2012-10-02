cmake_minimum_required(VERSION 2.8)
project(image_pipeline)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED camera_calibration  depth_image_proc  image_proc  image_rotate  image_view  stereo_image_proc)
catkin_package(DEPENDS camera_calibration  depth_image_proc  image_proc  image_rotate  image_view  stereo_image_proc)

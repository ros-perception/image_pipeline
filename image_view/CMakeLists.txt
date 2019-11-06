cmake_minimum_required(VERSION 3.5)
project(image_view)

find_package(camera_calibration_parsers REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(stereo_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Extra tools
add_executable(extract_images
  src/nodes/extract_images.cpp)
target_link_libraries(extract_images
  ${OpenCV_LIBRARIES}
)

add_executable(image_saver
  src/nodes/image_saver.cpp)
target_link_libraries(image_saver
  ${OpenCV_LIBRARIES}
)

add_executable(video_recorder
  src/nodes/video_recorder.cpp)
target_link_libraries(video_recorder
  ${OpenCV_LIBRARIES}
)

install(TARGETS extract_images image_saver video_recorder
  DESTINATION bin
)

# Deal with the GUI's
if(ANDROID)
  return()
endif()

find_package(GTK2)
add_definitions(-DHAVE_GTK)
include_directories(${GTK2_INCLUDE_DIRS})

add_library(image_view_node SHARED
  src/nodelets/image_nodelet.cpp
  src/nodelets/disparity_nodelet.cpp
  src/nodelets/window_thread.cpp)
target_link_libraries(image_view_node
  ${GTK_LIBRARIES}
  ${GTK2_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
install(TARGETS image_view_node
  DESTINATION lib
)

# Image viewers
add_executable(image_view
  src/nodes/image_view.cpp)
target_link_libraries(image_view
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(disparity_view
  src/nodes/disparity_view.cpp)
target_link_libraries(disparity_view
  ${OpenCV_LIBRARIES}
)

add_executable(stereo_view
  src/nodes/stereo_view.cpp)
target_link_libraries(stereo_view
  ${Boost_LIBRARIES}
  ${GTK_LIBRARIES}
  ${GTK2_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS disparity_view image_view stereo_view
  DESTINATION bin 
)

# Python programs
#catkin_install_python(
#  PROGRAMS scripts/extract_images_sync
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

ament_package()

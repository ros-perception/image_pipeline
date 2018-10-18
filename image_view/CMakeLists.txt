cmake_minimum_required(VERSION 3.5)

project(image_view)


# ROS2 Flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTK2)
find_package(std_srvs REQUIRED)
find_package(rcutils)
find_package(camera_calibration_parsers)
add_definitions(-DHAVE_GTK)
include_directories(${GTK2_INCLUDE_DIRS})

if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
  add_compile_options(-std=c++11)
endif()

add_executable(image_view_exe src/nodes/image_view.cpp)
SET_TARGET_PROPERTIES(image_view_exe PROPERTIES OUTPUT_NAME image_view)
ament_target_dependencies(image_view_exe
  image_transport
  image_geometry
  cv_bridge
  message_filters
  rclcpp
  rcutils
  gtk
  stereo_msgs
  sensor_msgs
  rmw_implementation
  camera_calibration_parsers
  cv_bridge
  OpenCV
)
install(TARGETS image_view_exe
        DESTINATION bin
)

add_executable(stereo_view src/nodes/stereo_view.cpp)
ament_target_dependencies(stereo_view
  image_transport
  image_geometry
  cv_bridge
  message_filters
  rclcpp
  gtk
  rcutils
  stereo_msgs
  sensor_msgs
  rmw_implementation
  cv_bridge
  OpenCV
)

add_executable(extract_images src/nodes/extract_images.cpp)
ament_target_dependencies(extract_images
  image_transport
  image_geometry
  cv_bridge
  message_filters
  rclcpp
  rcutils
  stereo_msgs
  sensor_msgs
  rmw_implementation
  cv_bridge
  OpenCV
  Boost
)

add_executable(image_saver src/nodes/image_saver.cpp)
ament_target_dependencies(image_saver
  image_transport
  image_geometry
  cv_bridge
  camera_calibration_parsers
  std_srvs
  message_filters
  rclcpp
  rcutils
  stereo_msgs
  sensor_msgs
  rmw_implementation
  cv_bridge
  OpenCV
  Boost
)

add_executable(video_recorder src/nodes/video_recorder.cpp)
ament_target_dependencies(video_recorder
  image_transport
  image_geometry
  cv_bridge
  std_srvs
  rclcpp
  rcutils
  sensor_msgs
  rmw_implementation
  cv_bridge
  OpenCV
  Boost
)

add_executable(disparity_view
               src/nodes/disparity_view.cpp
               src/nodes/window_thread.cpp)
ament_target_dependencies(disparity_view
  image_transport
  image_geometry
  cv_bridge
  std_srvs
  gtk
  rclcpp
  rcutils
  sensor_msgs
  stereo_msgs
  rmw_implementation
  cv_bridge
  OpenCV
  Boost
)
install(TARGETS 
        image_view_exe
        stereo_view
        extract_images
        image_saver
        video_recorder
        disparity_view
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY examples/launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()

cmake_minimum_required(VERSION 3.5)
project(image_view)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(stereo_msgs REQUIRED)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Deal with the GUI's
if(ANDROID)
  return()
endif()

add_library(image_view_nodes SHARED
  src/disparity_view_node.cpp
  src/extract_images_node.cpp
  src/image_view_node.cpp
  src/image_saver_node.cpp
  src/stereo_view_node.cpp
  src/video_recorder_node.cpp
)
ament_target_dependencies(image_view_nodes
  camera_calibration_parsers
  cv_bridge
  image_transport
  rclcpp
  rclcpp_components
  sensor_msgs
  std_srvs
  stereo_msgs
)
target_link_libraries(image_view_nodes
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_compile_definitions(image_view_nodes
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_nodes(image_view_nodes
  "image_view::DisparityViewNode"
)
rclcpp_components_register_nodes(image_view_nodes
  "image_view::ExtractImagesNode"
)
rclcpp_components_register_nodes(image_view_nodes
  "image_view::ImageViewNode"
)
rclcpp_components_register_nodes(image_view_nodes
  "image_view::ImageSaverNode"
)
rclcpp_components_register_nodes(image_view_nodes
  "image_view::StereoViewNode"
)
rclcpp_components_register_nodes(image_view_nodes
  "image_view::VideoRecorderNode"
)
ament_export_include_directories(include)
ament_export_interfaces(export_image_view_nodes)
ament_export_libraries(image_view_nodes)

# Image viewers
add_executable(disparity_view
  src/disparity_view.cpp
)
ament_target_dependencies(disparity_view
  rclcpp
)
target_link_libraries(disparity_view
  image_view_nodes
)
add_dependencies(disparity_view
  image_view_nodes
)

add_executable(image_view
  src/image_view.cpp
)
ament_target_dependencies(image_view
  rclcpp
)
target_link_libraries(image_view
  image_view_nodes
)
add_dependencies(image_view
  image_view_nodes
)

add_executable(stereo_view
  src/stereo_view.cpp
)
ament_target_dependencies(stereo_view
  rclcpp
)
target_link_libraries(stereo_view
  image_view_nodes
)
add_dependencies(stereo_view
  image_view_nodes
)

# Other Tools
add_executable(extract_images
  src/extract_images.cpp
)
ament_target_dependencies(extract_images
  rclcpp
)
target_link_libraries(extract_images
  image_view_nodes
)
add_dependencies(extract_images
  image_view_nodes
)

add_executable(image_saver
  src/image_saver.cpp
)
ament_target_dependencies(image_saver
  rclcpp
)
target_link_libraries(image_saver
  image_view_nodes
)
add_dependencies(image_saver
  image_view_nodes
)

add_executable(video_recorder
  src/video_recorder.cpp
)
ament_target_dependencies(video_recorder
  rclcpp
)
target_link_libraries(video_recorder
  image_view_nodes
)
add_dependencies(video_recorder
  image_view_nodes
)

if(BUILD_TESTING)
  find_package(ament_lint_auto)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS
    disparity_view
    extract_images
    image_saver
    image_view
    stereo_view
    video_recorder
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS image_view_nodes
  EXPORT export_image_view_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Python programs
#catkin_install_python(
#  PROGRAMS scripts/extract_images_sync
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

ament_package()

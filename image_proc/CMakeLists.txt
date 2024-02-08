cmake_minimum_required(VERSION 3.5)
project(image_proc)

# ROS2 Flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
if(OpenCV_VERSION VERSION_LESS "3.2.0")
  message(FATAL "Minimum OpenCV version is 3.2.0 (found version ${OpenCV_VERSION})")
endif()

# image_proc library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/processor.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

# rectify component and node
ament_auto_add_library(rectify SHARED
  src/rectify.cpp)
target_compile_definitions(rectify
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_node(rectify
  PLUGIN "image_proc::RectifyNode"
  EXECUTABLE rectify_node
)

# debayer component and node
ament_auto_add_library(debayer SHARED
  src/debayer.cpp
  src/edge_aware.cpp
)
target_compile_definitions(debayer
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_node(debayer
  PLUGIN "image_proc::DebayerNode"
  EXECUTABLE debayer_node
)

# resize component and node
ament_auto_add_library(resize SHARED
  src/resize.cpp
)
target_compile_definitions(resize
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_node(resize
  PLUGIN "image_proc::ResizeNode"
  EXECUTABLE resize_node
)

# crop_decimate component and node
ament_auto_add_library(crop_decimate SHARED
  src/crop_decimate.cpp
)
target_compile_definitions(crop_decimate
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_node(crop_decimate
  PLUGIN "image_proc::CropDecimateNode"
  EXECUTABLE crop_decimate_node
)

# crop_non_zero component and node
ament_auto_add_library(crop_non_zero SHARED
  src/crop_non_zero.cpp
)
target_compile_definitions(crop_non_zero
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_node(crop_non_zero
  PLUGIN "image_proc::CropNonZeroNode"
  EXECUTABLE crop_non_zero_node
)

# track_marker component and node
ament_auto_add_library(track_marker SHARED
  src/track_marker.cpp
)
target_compile_definitions(track_marker
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_node(track_marker
  PLUGIN "image_proc::TrackMarkerNode"
  EXECUTABLE track_marker_node
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_libraries(image_proc "stdc++fs")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_definitions(-D_SRC_RESOURCES_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/resources")

  find_package(ament_cmake_gtest)
  ament_auto_add_gtest(test_rectify test/test_rectify.cpp)

  find_package(ament_cmake_gtest)
  ament_auto_add_gtest(rostest test/rostest.cpp)
endif()

ament_auto_package(INSTALL_TO_SHARE launch)

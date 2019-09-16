cmake_minimum_required(VERSION 3.5)

project(image_proc)

# ROS2 Flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_geometry REQUIRED)

include_directories(include)

add_library(rectify SHARED
  src/rectify.cpp)
target_compile_definitions(rectify
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(rectify
  "image_geometry"
  "sensor_msgs"
  "cv_bridge"
  "class_loader"
  "image_transport"
  "rclcpp"
  "rclcpp_components"
  )
  
rclcpp_components_register_nodes(rectify "image_proc::RectifyNode")
set(node_plugins "${node_plugins}image_proc::RectifyNode;$<TARGET_FILE:rectify>\n")

add_library(debayer SHARED
  src/debayer.cpp
  src/edge_aware.cpp
  )

target_compile_definitions(debayer
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(debayer
  "sensor_msgs"
  "cv_bridge"
  "class_loader"
  "image_transport"
  "rclcpp"
  "rclcpp_components"
  )

rclcpp_components_register_nodes(debayer "image_proc::DebayerNode")
set(node_plugins "${node_plugins}image_proc::DebayerNode;$<TARGET_FILE:debayer>\n")

add_executable(image_proc
               src/image_proc.cpp
)
target_link_libraries(image_proc
  debayer
  rectify
  ament_index_cpp::ament_index_cpp)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_libraries(image_proc "stdc++fs")
endif()

ament_target_dependencies(image_proc
  "image_geometry"
  "sensor_msgs"
  "cv_bridge"
  "class_loader"
  "image_transport"
  "rclcpp"
  "rclcpp_components"
  "rcutils"
)

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

install(TARGETS
  debayer
  rectify
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

install(TARGETS
        image_proc
        DESTINATION lib/${PROJECT_NAME})
ament_package()
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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_geometry REQUIRED)

if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
  add_compile_options(-std=c++11)
endif()

add_executable(image_proc
               src/image_proc.cpp
               src/debayer.cpp
               src/rectify.cpp
               src/edge_aware.cpp)
target_link_libraries(image_proc ament_index_cpp::ament_index_cpp)

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
  "rcutils"
)


install(TARGETS
        image_proc
        DESTINATION lib/${PROJECT_NAME})
ament_package()
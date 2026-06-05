cmake_minimum_required(VERSION 3.20)
project(image_publisher)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(camera_info_manager REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs videoio)
message(STATUS "opencv version ${OpenCV_VERSION}")

add_library(image_publisher SHARED src/image_publisher.cpp)
target_include_directories(image_publisher PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(image_publisher PUBLIC
  ${OpenCV_LIBRARIES}
  camera_info_manager::camera_info_manager
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  rclcpp_components::component
  ${rcl_interfaces_TARGETS})
rclcpp_components_register_nodes(image_publisher "${PROJECT_NAME}::ImagePublisher")
set(node_plugins "${node_plugins}${PROJECT_NAME}::ImagePublisher;$<TARGET_FILE:ImagePublisher>\n")

install(
  TARGETS image_publisher
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  OpenCV
  camera_info_manager
  cv_bridge
  image_transport
  rcl_interfaces
  rclcpp
  rclcpp_components)

ament_auto_add_executable(image_publisher_node src/image_publisher_node.cpp)
target_link_libraries(image_publisher_node image_publisher)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)

cmake_minimum_required(VERSION 3.5)
project(image_flip)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED core imgproc)

ament_auto_add_library(${PROJECT_NAME} SHARED src/image_flip_node.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "${PROJECT_NAME}::ImageFlipNode"
  EXECUTABLE image_flip_node
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)

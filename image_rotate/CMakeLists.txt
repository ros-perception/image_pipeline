cmake_minimum_required(VERSION 3.5)
project(image_rotate)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED core imgproc)

# add the executable
include_directories(include)

add_library(${PROJECT_NAME} SHARED src/image_rotate_node.cpp)
target_link_libraries(${PROJECT_NAME} ${ament_LIBRARIES} ${OpenCV_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}
  "image_transport"
  "cv_bridge"
  "class_loader"
  "rclcpp_components"
  "rclcpp"
  "rclcpp_components"
  "tf2_ros"
  "tf2_geometry_msgs"
)
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::ImageRotateNode")
set(node_plugins "${node_plugins}${PROJECT_NAME}::ImageRotateNode;$<TARGET_FILE:ImageRotateNode>\n")

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib
)

add_executable(image_rotate_bin src/image_rotate.cpp)
set_target_properties(image_rotate_bin PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_link_libraries(image_rotate_bin ${ament_LIBRARIES} ${OpenCV_LIBRARIES} ${PROJECT_NAME})
ament_target_dependencies(image_rotate_bin
  "rclcpp")

install(TARGETS image_rotate_bin
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

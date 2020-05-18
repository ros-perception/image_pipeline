cmake_minimum_required(VERSION 3.5)
project(image_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_geometry REQUIRED)
find_package(rclcpp_components REQUIRED)

set( DEPENDENCIES
  rclcpp
  rclcpp_components
  image_geometry
  class_loader
  image_transport
  camera_info_manager
  cv_bridge
)

find_package(OpenCV REQUIRED COMPONENTS core)
message(STATUS "opencv version ${OpenCV_VERSION}")
find_package(OpenCV 4 REQUIRED COMPONENTS ${opencv_4_components})

include_directories(include)

add_library(image_publisher SHARED src/image_publisher.cpp)
target_link_libraries(image_publisher ${ament_LIBRARIES} ${OpenCV_LIBRARIES})
ament_target_dependencies(image_publisher
  ${DEPENDENCIES}
)

rclcpp_components_register_nodes(image_publisher "${PROJECT_NAME}::ImagePublisher")
set(node_plugins "${node_plugins}${PROJECT_NAME}::ImagePublisher;$<TARGET_FILE:ImagePublisher>\n")

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h")

add_executable(image_publisher_node src/image_publisher_node.cpp)
target_link_libraries(image_publisher_node ${ament_LIBRARIES} image_publisher)
ament_target_dependencies(image_publisher_node
  ${DEPENDENCIES})

install(TARGETS image_publisher_node
        DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS image_publisher
        DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS image_publisher
        DESTINATION bin
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${DEPENDENCIES})
ament_package()

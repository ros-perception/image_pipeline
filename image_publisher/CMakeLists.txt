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

set(opencv_2_components core highgui)
set(opencv_3_components core imgcodecs videoio)
find_package(OpenCV REQUIRED COMPONENTS core)
message(STATUS "opencv version ${OpenCV_VERSION}")
if(OpenCV_VERSION VERSION_LESS "3.0.0")
  find_package(OpenCV 2 REQUIRED COMPONENTS ${opencv_2_components})
else()
  find_package(OpenCV 3 REQUIRED COMPONENTS ${opencv_3_components})
endif()

include_directories(include)

add_library(image_publisher_node SHARED src/nodelet/image_publisher_nodelet.cpp)
target_link_libraries(image_publisher_node ${ament_LIBRARIES} ${OpenCV_LIBRARIES})
ament_target_dependencies(image_publisher_node
  "image_transport"
  "image_geometry"
  "cv_bridge"
  "class_loader"
  "camera_info_manager"
  "rclcpp"
)

rclcpp_register_node_plugins(image_publisher_node "${PROJECT_NAME}::ImagePublisherNode")
set(node_plugins "${node_plugins}${PROJECT_NAME}::ImagePublisherNode;$<TARGET_FILE:ImagePublisherNode>\n")

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h")

install(TARGETS image_publisher_node
        DESTINATION lib
)

add_executable(image_publisher src/node/image_publisher.cpp)
target_link_libraries(image_publisher ${ament_LIBRARIES} image_publisher_node)
ament_target_dependencies(image_publisher
  "rclcpp")

install(TARGETS image_publisher
        DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS image_publisher
        DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

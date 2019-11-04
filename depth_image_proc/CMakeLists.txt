cmake_minimum_required(VERSION 3.5)
project(depth_image_proc)

# Default to C++14
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
find_package(tf2_eigen REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(class_loader REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()
find_package(OpenCV REQUIRED)
include_directories(include ${BOOST_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/convert_metric.cpp
  src/crop_foremost.cpp
  src/disparity.cpp
  src/point_cloud_xyz.cpp
  src/point_cloud_xyzrgb.cpp
  src/point_cloud_xyzi.cpp
  src/point_cloud_xyz_radial.cpp
  src/point_cloud_xyzi_radial.cpp
  src/register.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  "image_transport"
  "image_geometry"
  "cv_bridge"
  "class_loader"
  "message_filters"
  "rclcpp"
  "rclcpp_components"
  "tf2_ros"
  "tf2"
  "tf2_eigen"
  "stereo_msgs"
  "sensor_msgs"
)

rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::ConvertMetricNode")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::CropForemostNode")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::DisparityNode")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::PointCloudXyzNode")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::PointCloudXyzRadialNode")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::PointCloudXyziNode")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::PointCloudXyziRadialNode")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::PointCloudXyzrgbNode")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::RegisterNode")

target_link_libraries(${PROJECT_NAME} ${ament_LIBRARIES} ${OpenCV_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h")

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

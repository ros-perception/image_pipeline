cmake_minimum_required(VERSION 3.5)
project(depth_image_proc)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  include_directories(include ${EIGEN3_INCLUDE_DIRS})
endif()

find_package(OpenCV REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/conversions.cpp
  src/convert_metric.cpp
  src/crop_foremost.cpp
  src/disparity.cpp
  src/point_cloud_xyz.cpp
  src/point_cloud_xyzrgb.cpp
  src/point_cloud_xyzi.cpp
  src/point_cloud_xyz_radial.cpp
  src/point_cloud_xyzi_radial.cpp
  src/point_cloud_xyzrgb_radial.cpp
  src/register.cpp
)

# Register individual components and also build standalone nodes for each
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "depth_image_proc::ConvertMetricNode"
  EXECUTABLE convert_metric_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "depth_image_proc::CropForemostNode"
  EXECUTABLE crop_foremost_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "depth_image_proc::DisparityNode"
  EXECUTABLE disparity_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "depth_image_proc::PointCloudXyzNode"
  EXECUTABLE point_cloud_xyz_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "depth_image_proc::PointCloudXyzrgbNode"
  EXECUTABLE point_cloud_xyzrgb_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "depth_image_proc::PointCloudXyziNode"
  EXECUTABLE point_cloud_xyzi_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "depth_image_proc::PointCloudXyzRadialNode"
  EXECUTABLE point_cloud_xyz_radial_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "depth_image_proc::PointCloudXyziRadialNode"
  EXECUTABLE point_cloud_xyzi_radial_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "depth_image_proc::PointCloudXyziRadialNode"
  EXECUTABLE point_cloud_xyzrgb_radial_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "depth_image_proc::RegisterNode"
  EXECUTABLE register_node
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)

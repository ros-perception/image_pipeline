cmake_minimum_required(VERSION 3.5)
project(stereo_image_proc)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_proc REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# See note in image_proc/CMakeLists.txt
# add_definitions(-DOPENCV_TRAITS_ENABLE_DEPRECATED)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/stereo_processor.cpp
  src/${PROJECT_NAME}/disparity_node.cpp
  src/${PROJECT_NAME}/point_cloud_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  cv_bridge
  image_geometry
  image_proc
  image_transport
  message_filters
  rclcpp
  rclcpp_components
  sensor_msgs
  stereo_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "stereo_image_proc::DisparityNode"
  EXECUTABLE disparity_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "stereo_image_proc::PointCloudNode"
  EXECUTABLE point_cloud_node
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(python_cmake_module REQUIRED)

  ament_lint_auto_find_test_dependencies()

  set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
  if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
  endif()

  # Test DisparityNode in launch test
  add_launch_test("test/test_disparity_node.py"
    PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
  )
  # TODO(jacobperron): As of Eloquent, they're just plain pytests
  # ament_add_pytest_test("test_disparity_node" test/test_disparity_node.py
  #   PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
  # )

  # Test PointCloudNode in launch test
  add_launch_test("test/test_point_cloud_node.py"
    PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
  )
  # ament_add_pytest_test("test_point_cloud_node" test/test_point_cloud_node.py
  #   PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
  # )

  set(PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}")
endif()

ament_package()

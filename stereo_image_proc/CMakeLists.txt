cmake_minimum_required(VERSION 3.5)
project(stereo_image_proc)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

# See note in image_proc/CMakeLists.txt
# add_definitions(-DOPENCV_TRAITS_ENABLE_DEPRECATED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/stereo_processor.cpp
  src/${PROJECT_NAME}/disparity_node.cpp
  src/${PROJECT_NAME}/point_cloud_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "stereo_image_proc::DisparityNode"
  EXECUTABLE disparity_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "stereo_image_proc::PointCloudNode"
  EXECUTABLE point_cloud_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
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

ament_auto_package(INSTALL_TO_SHARE launch)

cmake_minimum_required(VERSION 3.5)
project(image_proc)

# ROS2 Flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
if(OpenCV_VERSION VERSION_LESS "3.2.0")
  message(FATAL "Minimum OpenCV version is 3.2.0 (found version ${OpenCV_VERSION})")
endif()

# image_proc library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/processor.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

# rectify component and node
ament_auto_add_library(rectify SHARED
  src/rectify.cpp)
target_compile_definitions(rectify
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_node(rectify
  PLUGIN "image_proc::RectifyNode"
  EXECUTABLE rectify_node
)

# debayer component and node
ament_auto_add_library(debayer SHARED
  src/debayer.cpp
  src/edge_aware.cpp
)
target_compile_definitions(debayer
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_node(debayer
  PLUGIN "image_proc::DebayerNode"
  EXECUTABLE debayer_node
)

# resize component and node
ament_auto_add_library(resize SHARED
  src/resize.cpp
)
target_compile_definitions(resize
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_node(resize
  PLUGIN "image_proc::ResizeNode"
  EXECUTABLE resize_node
)

# crop_decimate component and node
ament_auto_add_library(crop_decimate SHARED
  src/crop_decimate.cpp
)
target_compile_definitions(crop_decimate
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_node(crop_decimate
  PLUGIN "image_proc::CropDecimateNode"
  EXECUTABLE crop_decimate_node
)

# crop_non_zero component and node
ament_auto_add_library(crop_non_zero SHARED
  src/crop_non_zero.cpp
)
target_compile_definitions(crop_non_zero
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_node(crop_non_zero
  PLUGIN "image_proc::CropNonZeroNode"
  EXECUTABLE crop_non_zero_node
)

# image_proc example node
ament_auto_add_executable(image_proc_exe
  src/image_proc.cpp
)
target_link_libraries(image_proc_exe
  debayer
  rectify
  ament_index_cpp::ament_index_cpp
)
set_target_properties(image_proc_exe PROPERTIES OUTPUT_NAME image_proc)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_libraries(image_proc "stdc++fs")
endif()

install(TARGETS image_proc_exe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)

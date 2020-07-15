cmake_minimum_required(VERSION 3.5)

project(image_proc)

# ROS2 Flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
if(OpenCV_VERSION VERSION_LESS "3.2.0")
  message(FATAL "Minimum OpenCV version is 3.2.0 (found version ${OpenCV_VERSION})")
endif()
find_package(image_geometry REQUIRED)

include_directories(
  include
)

set(dependencies
  image_geometry
  sensor_msgs
  cv_bridge
  class_loader
  image_transport
  rclcpp
  rclcpp_components
  rcutils
)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/processor.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  image_geometry
  rcutils
  sensor_msgs
)

add_library(rectify SHARED
  src/rectify.cpp)
target_compile_definitions(rectify
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(rectify
  ${dependencies}
)

rclcpp_components_register_nodes(rectify "image_proc::RectifyNode")
set(node_plugins "${node_plugins}image_proc::RectifyNode;$<TARGET_FILE:rectify>\n")

add_library(debayer SHARED
  src/debayer.cpp
  src/edge_aware.cpp
)

target_compile_definitions(debayer
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(debayer
  ${dependencies}
)

rclcpp_components_register_nodes(debayer "image_proc::DebayerNode")
set(node_plugins "${node_plugins}image_proc::DebayerNode;$<TARGET_FILE:debayer>\n")

add_library(resize SHARED
  src/resize.cpp
)

target_compile_definitions(resize
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(resize
  ${dependencies}
)

rclcpp_components_register_nodes(resize "image_proc::ResizeNode")
set(node_plugins "${node_plugins}image_proc::ResizeNode;$<TARGET_FILE:resize>\n")

add_library(crop_decimate SHARED
  src/crop_decimate.cpp
)

target_compile_definitions(crop_decimate
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(crop_decimate
  ${dependencies}
)

rclcpp_components_register_nodes(crop_decimate "image_proc::CropDecimateNode")
set(node_plugins "${node_plugins}image_proc::CropDecimateNode;$<TARGET_FILE:crop_decimate>\n")
add_library(crop_non_zero SHARED
  src/crop_non_zero.cpp
)

target_compile_definitions(crop_non_zero
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(crop_non_zero
  ${dependencies}
)

rclcpp_components_register_nodes(crop_non_zero "image_proc::CropNonZeroNode")
set(node_plugins "${node_plugins}image_proc::CropNonZeroNode;$<TARGET_FILE:crop_non_zero>\n")

add_executable(image_proc_exe
  src/image_proc.cpp
)

target_link_libraries(image_proc_exe
  debayer
  rectify
  ament_index_cpp::ament_index_cpp
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_libraries(image_proc "stdc++fs")
endif()

ament_target_dependencies(image_proc_exe
  ${dependencies}
)

set_target_properties(image_proc_exe PROPERTIES OUTPUT_NAME image_proc)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS image_proc_exe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS crop_decimate
  EXPORT export_crop_decimate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS crop_non_zero
  EXPORT export_crop_non_zero
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS debayer
  EXPORT export_debayer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS rectify
  EXPORT export_rectify
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS resize
  EXPORT export_resize
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_export_interfaces(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
  export_crop_decimate HAS_LIBRARY_TARGET
  export_crop_non_zero HAS_LIBRARY_TARGET
  export_debayer HAS_LIBRARY_TARGET
  export_rectify HAS_LIBRARY_TARGET
  export_resize HAS_LIBRARY_TARGET
)

ament_export_libraries(
  ${PROJECT_NAME}
  crop_decimate
  crop_non_zero
  debayer
  rectify
  resize
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

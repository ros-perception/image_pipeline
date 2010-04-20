cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include(FindPkgConfig)
if (PKG_CONFIG_FOUND AND UNIX AND NOT APPLE)
   pkg_check_modules(GTK gtk+-2.0)
   if (GTK_FOUND)
      include_directories(${GTK_INCLUDE_DIRS})
      link_directories(${GTK_LIBRARY_DIRS})
      add_definitions(-DHAVE_GTK)
   endif(GTK_FOUND)
endif(PKG_CONFIG_FOUND AND UNIX AND NOT APPLE)

# Image viewers
rosbuild_add_executable(image_view src/image_view.cpp)
target_link_libraries(image_view ${GTK_LIBRARIES})

rosbuild_add_executable(stereo_view src/stereo_view.cpp)
target_link_libraries(stereo_view ${GTK_LIBRARIES})

# Extra tools (unstable)
rosbuild_add_executable(extract_images src/extract_images.cpp)

rosbuild_add_executable(image_saver src/image_saver.cpp)

# Hack to remove binaries previously built in the package root.
execute_process(COMMAND cmake -E remove ${PROJECT_SOURCE_DIR}/image_view ${PROJECT_SOURCE_DIR}/stereo_view ${PROJECT_SOURCE_DIR}/extract_images ${PROJECT_SOURCE_DIR}/image_saver)

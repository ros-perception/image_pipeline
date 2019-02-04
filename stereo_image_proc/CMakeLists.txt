cmake_minimum_required(VERSION 2.8)
project(stereo_image_proc)

find_package(catkin REQUIRED cv_bridge dynamic_reconfigure image_geometry image_proc image_transport message_filters nodelet sensor_msgs stereo_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

##DEBUGGING BUILD
#set(CMAKE_VERBOSE_MAKEFILE TRUE)

# CUDA library loading
#
# Requires: pkg-config
#
# Try to load package-config. If it doesn't exist, then we'll just fail to load CUDA libraries, no big deal.
set(all_visionworks_INCLUDE_DIRS "")
set(all_visionworks_LIBRARIES "")
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  #Prefer CUDA 9.0, then 9.1
  pkg_check_modules(CUDART cudart-9.0)
  pkg_check_modules(CORE_CUDA cuda-9.0)
  if(CUDART_FOUND AND CORE_CUDA_FOUND)
    message(STATUS "Found CUDA 9.0")
  else()
    pkg_check_modules(CUDART cudart-9.1)
    pkg_check_modules(CORE_CUDA cuda-9.1)
    if(CUDART_FOUND AND CORE_CUDA_FOUND)
      message(STATUS "Found CUDA 9.1")
    endif()
  endif()
  if(NOT CUDART_FOUND OR NOT CORE_CUDA_FOUND)
    message(STATUS "No CUDA found.")
  else()
    pkg_check_modules(VISIONWORKS visionworks)
    if(VISIONWORKS_FOUND)
      # Note about NVXLIB AND OVXLIB
      # These libraries normally compile statically, without PIC (Position Independent Code). This means that they cannot be
      # linked into a .so!
      # Normally, this is fine, because we'd link them directly into the end binary. However, we're using nodelets, which
      # are loaded at runtime into the "nodelet" binary, which is not linked against NVX or OVX.
      # Thus, we need them in the .so. So, you MUST compile VisionWorks sources with the CXX flag "-fPIC"!
      find_library(NVXLIB nvx PATHS ${VISIONWORKS_LIBRARY_DIRS})
      find_library(OVXLIB ovx PATHS ${VISIONWORKS_LIBRARY_DIRS})
      if(NOT NVXLIB OR NOT OVXLIB)
        message(WARNING "Found visionworks, but didn't find libovx or libnvx! You probably need to compile the visionworks sources!")
      else()
        message(STATUS "NVX library: ${NVXLIB}")
        message(STATUS "OVX library: ${OVXLIB}")
      endif()
    else()
      message(WARNING "Found CUDA, but could not find VisionWorks")
    endif()
  endif()
  
  if(CUDART_FOUND AND CORE_CUDA_FOUND AND VISIONWORKS_FOUND AND NVXLIB AND OVXLIB)
    message(STATUS "Found acceptable CUDA and VisionWorks on this system. Will use GPU acceleration.")
    set(all_visionworks_FOUND TRUE)
    add_definitions(-DVISIONWORKS_ACCELERATION)
  endif()
else()
  message(STATUS "Could not find pkg-config, so will not try to use CUDA acceleration.")
endif()

if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
  add_compile_options(-std=c++14)
endif()

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/Disparity.cfg)

catkin_package(
  CATKIN_DEPENDS image_geometry image_proc sensor_msgs stereo_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include)

find_package(OpenCV REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${all_visionworks_INCLUDE_DIRS})
if(all_visionworks_FOUND)
  include_directories(${CUDART_INCLUDE_DIRS} ${CORE_CUDA_INCLUDE_DIRS} ${VISIONWORKS_INCLUDE_DIRS})
endif()

# See note in image_proc/CMakeLists.txt
add_definitions(-DOPENCV_TRAITS_ENABLE_DEPRECATED)

# Nodelet library
if(all_visionworks_FOUND)
  add_library(${PROJECT_NAME}_visionworks
    src/libvisionworks_image_proc/visionworks_interface.cpp
    src/libvisionworks_image_proc/visionworks_cv_helpers.cpp
    src/libvisionworks_image_proc/speckle_filter_node.cpp
    src/libvisionworks_image_proc/masking_node.cpp
    src/libvisionworks_image_proc/sobel_filter_node.cpp)
  target_link_libraries(${PROJECT_NAME}_visionworks ${catkin_LIBRARIES}
                        ${OpenCV_LIBRARIES}
                        ${CUDART_LDFLAGS}
                        ${CORE_CUDA_LDFLAGS}
                        ${VISIONWORKS_LDFLAGS}
                        ${NVXLIB}
                        ${OVXLIB}
                        )

  add_library(${PROJECT_NAME} src/libstereo_image_proc/processor.cpp src/libvisionworks_image_proc/visionworks_matching.cpp src/nodelets/disparity.cpp src/nodelets/point_cloud2.cpp)  
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                        ${OpenCV_LIBRARIES}
                        ${PROJECT_NAME}_visionworks
                        )
else()
  add_library(${PROJECT_NAME} src/libstereo_image_proc/processor.cpp src/nodelets/disparity.cpp src/nodelets/point_cloud2.cpp)
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                        ${OpenCV_LIBRARIES}
                                        )
endif()
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Standalone node
add_executable(stereoimageproc_exe src/nodes/stereo_image_proc.cpp)
target_link_libraries(stereoimageproc_exe ${PROJECT_NAME})
SET_TARGET_PROPERTIES(stereoimageproc_exe PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
install(TARGETS stereoimageproc_exe
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install the launch file
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

# install the include directory
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

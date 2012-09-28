cmake_minimum_required(VERSION 2.8)
project(image_proc)

find_package(catkin REQUIRED)
catkin_package()

find_package(catkin REQUIRED image_geometry)
include_directories(SYSTEM ${image_geometry_INCLUDE_DIRS})
include_directories(include)

# Dynamic reconfigure support
if (FALSE)
find_package(catkin REQUIRED dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
endif()

# Nodelet library
if (FALSE)
add_library(image_proc src/libimage_proc/processor.cpp
                                src/nodelets/debayer.cpp
                                src/nodelets/rectify.cpp
                                src/nodelets/crop_decimate.cpp
                                src/libimage_proc/advertisement_checker.cpp
                                src/nodelets/edge_aware.cpp
				src/nodelets/yuv422.cpp
)
install(TARGETS image_proc
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

# Standalone node
add_executable(image_proc_exe src/nodes/image_proc.cpp)
target_link_libraries(image_proc_exe image_proc)
SET_TARGET_PROPERTIES(image_proc_exe PROPERTIES OUTPUT_NAME image_proc)
install(TARGETS image_proc_exe
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endif()

# Tests
catkin_add_gtest(image_proc_rostest test/rostest.cpp)

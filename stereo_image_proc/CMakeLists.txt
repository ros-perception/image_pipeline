project(stereo_image_proc)
catkin_project(stereo_image_proc)

include_directories(include)
return()

# Dynamic reconfigure support
find_package(catkin REQUIRED dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# Nodelet library
add_library(stereo_image_proc src/libstereo_image_proc/processor.cpp src/nodelets/disparity.cpp src/nodelets/point_cloud2.cpp src/nodelets/point_cloud.cpp)
install(TARGETS stereo_image_proc
        DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION}
)

# Standalone node
add_executable(stereoimageproc_exe src/nodes/stereo_image_proc.cpp)
target_link_libraries(stereoimageproc_exe stereo_image_proc)
SET_TARGET_PROPERTIES(stereoimageproc_exe PROPERTIES OUTPUT_NAME stereo_image_proc)
install(TARGETS stereoimageproc_exe
        DESTINATION ${CATKIN_PROJECT_BIN_DESTINATION}
)

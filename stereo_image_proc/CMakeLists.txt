cmake_minimum_required(VERSION 2.8)
project(stereo_image_proc)

find_package(catkin REQUIRED cv_bridge dynamic_reconfigure image_geometry image_proc image_transport message_filters nodelet sensor_msgs stereo_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/Disparity.cfg)

catkin_package(
  CATKIN_DEPENDS image_geometry image_proc sensor_msgs stereo_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include)

find_package(OpenCV REQUIRED)
if (OpenCV_VERSION VERSION_EQUAL "3")
  add_definitions("-DOPENCV3=1")
endif()
include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)

# Nodelet library
add_library(${PROJECT_NAME} src/libstereo_image_proc/processor.cpp src/nodelets/disparity.cpp src/nodelets/point_cloud2.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Standalone node
add_executable(stereoimageproc_exe src/nodes/stereo_image_proc.cpp)
target_link_libraries(stereoimageproc_exe stereo_image_proc)
SET_TARGET_PROPERTIES(stereoimageproc_exe PROPERTIES OUTPUT_NAME stereo_image_proc)
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

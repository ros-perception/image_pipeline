cmake_minimum_required(VERSION 2.8)
project(image_proc)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED camera_calibration_parsers cv_bridge dynamic_reconfigure image_geometry image_transport nodelet roscpp sensor_msgs)
find_package(OpenCV REQUIRED)
if (OpenCV_VERSION VERSION_EQUAL "3")
  add_definitions("-DOPENCV3=1")
endif()
find_package(Boost REQUIRED COMPONENTS thread)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/CropDecimate.cfg cfg/Debayer.cfg cfg/Rectify.cfg)

catkin_package(
  CATKIN_DEPENDS image_geometry roscpp sensor_msgs
  DEPENDS OpenCV
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(include)

# Nodelet library
add_library(${PROJECT_NAME} src/libimage_proc/processor.cpp
                                src/nodelets/debayer.cpp
                                src/nodelets/rectify.cpp
                                src/nodelets/crop_decimate.cpp
                                src/libimage_proc/advertisement_checker.cpp
                                src/nodelets/edge_aware.cpp
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Standalone node
add_executable(image_proc_exe src/nodes/image_proc.cpp)
target_link_libraries(image_proc_exe ${PROJECT_NAME}  ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
SET_TARGET_PROPERTIES(image_proc_exe PROPERTIES OUTPUT_NAME image_proc)
install(TARGETS image_proc_exe
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install the launch file
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

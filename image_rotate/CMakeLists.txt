cmake_minimum_required(VERSION 2.8)
project(image_rotate)

find_package(catkin REQUIRED COMPONENTS cv_bridge dynamic_reconfigure image_transport roscpp tf)

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(cfg/ImageRotate.cfg)

catkin_package()

find_package(OpenCV REQUIRED)

# add the executable
include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/node/image_rotate.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

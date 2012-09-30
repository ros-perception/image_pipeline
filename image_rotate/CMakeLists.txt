cmake_minimum_required(VERSION 2.8)
project(image_rotate)

find_package(catkin REQUIRED)
catkin_package()

find_package(catkin REQUIRED dynamic_reconfigure tf)
gencfg()

find_package(OpenCV REQUIRED)

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS}
                           ${tf_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/node/image_rotate.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

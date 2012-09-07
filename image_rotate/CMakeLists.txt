project(image_rotate)
catkin_project(image_rotate)

return()

find_package(catkin REQUIRED dynamic_reconfigure tf)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

find_package(OpenCV REQUIRED)

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS}
                           ${tf_INCLUDE_DIRS}
)

add_executable(image_rotate src/node/image_rotate.cpp)
target_link_libraries(image_rotate ${OpenCV_LIBRARIES})
install(TARGETS image_rotate
        DESTINATION ${CATKIN_PROJECT_BIN_DESTINATION}
)

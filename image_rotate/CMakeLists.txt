cmake_minimum_required(VERSION 3.0.2)
project(image_rotate)

find_package(catkin REQUIRED COMPONENTS cmake_modules cv_bridge dynamic_reconfigure image_transport nodelet roscpp tf2 tf2_geometry_msgs)

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(cfg/ImageRotate.cfg)

catkin_package()

find_package(OpenCV REQUIRED core imgproc)

# add the executable
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED src/nodelet/image_rotate_nodelet.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
install(TARGETS image_rotate
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

add_executable(image_rotate_exe src/node/image_rotate.cpp)
SET_TARGET_PROPERTIES(image_rotate_exe PROPERTIES OUTPUT_NAME image_rotate)
target_link_libraries(image_rotate_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS image_rotate_exe
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

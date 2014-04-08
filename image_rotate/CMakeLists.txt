cmake_minimum_required(VERSION 2.8)
project(image_rotate)

find_package(catkin REQUIRED COMPONENTS cmake_modules cv_bridge dynamic_reconfigure eigen_conversions image_transport nodelet roscpp tf2 tf2_geometry_msgs)

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(cfg/ImageRotate.cfg)

catkin_package()

find_package(OpenCV REQUIRED core imgproc)
find_package(Eigen REQUIRED)

# add the executable
include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${EIGEN_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/nodelet/image_rotate_nodelet.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

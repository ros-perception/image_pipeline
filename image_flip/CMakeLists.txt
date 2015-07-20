cmake_minimum_required(VERSION 2.8.3)
project(image_flip)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

find_package(OpenCV)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_flip src/flip_image.cpp)
target_link_libraries(image_flip ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS image_flip
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

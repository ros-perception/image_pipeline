cmake_minimum_required(VERSION 2.8.3)
project(image_proc_fisheye)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  image_geometry
  OpenCV
  sensor_msgs
  std_msgs
  roscpp
  nodelet  
)

catkin_package(
  CATKIN_DEPENDS image_geometry roscpp sensor_msgs
  DEPENDS OpenCV
  LIBRARIES ${PROJECT_NAME}
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(include)

add_library(${PROJECT_NAME}     src/nodelets/rectify_.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(image_proc_fisheye_exe src/image_proc_fisheye.cpp)

target_link_libraries(image_proc_fisheye_exe
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

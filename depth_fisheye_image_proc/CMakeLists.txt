cmake_minimum_required(VERSION 2.8)
project(depth_fisheye_image_proc)

find_package(catkin REQUIRED cmake_modules cv_bridge eigen_conversions image_geometry image_transport message_filters nodelet sensor_msgs stereo_msgs tf2 tf2_ros)

if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
  add_compile_options(-std=c++11)
endif()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME})

find_package(Boost REQUIRED)
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()
find_package(OpenCV REQUIRED)
include_directories(include ${BOOST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
                             src/nodelets/point_cloud_xyz.cpp
                             #src/nodelets/point_cloud_xyzrgb.cpp


)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(depth_fisheye_image_proc_exe src/depth_fisheye_image_proc.cpp)


target_link_libraries(depth_fisheye_image_proc_exe
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
)


install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)











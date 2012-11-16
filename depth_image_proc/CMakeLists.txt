cmake_minimum_required(VERSION 2.8)
project(depth_image_proc)

find_package(catkin REQUIRED)
catkin_package(LIBRARIES ${PROJECT_NAME})

find_package(Boost REQUIRED)
find_package(catkin REQUIRED cv_bridge image_geometry image_transport message_filters nodelet pcl_ros sensor_msgs stereo_msgs tf)
find_package(Eigen REQUIRED)
find_package(PCL)
include_directories(SYSTEM ${BOOST_INCLUDE_DIRS}
                           ${catkin_INCLUDE_DIRS}
                           ${EIGEN_INCLUDE_DIRS}
                           ${PCL_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME} src/nodelets/convert_metric.cpp
                             src/nodelets/disparity.cpp
                             src/nodelets/point_cloud_xyz.cpp
                             src/nodelets/point_cloud_xyzrgb.cpp
                             src/nodelets/register.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

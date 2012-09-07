project(depth_image_proc)
catkin_project(depth_image_proc)

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
return()
add_library(${PROJECT_NAME} src/nodelets/convert_metric.cpp
                             src/nodelets/disparity.cpp
                             src/nodelets/point_cloud_xyz.cpp
                             src/nodelets/point_cloud_xyzrgb.cpp
                             src/nodelets/register.cpp
)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION}
)

cmake_minimum_required(VERSION 2.8)
project(image_view)

find_package(catkin REQUIRED)
catkin_package()

find_package(Boost REQUIRED COMPONENTS signals thread)
find_package(OpenCV REQUIRED)
if (OpenCV_VERSION VERSION_EQUAL "3")
  add_definitions("-DOPENCV3=1")
endif()

find_package(catkin REQUIRED camera_calibration_parsers cv_bridge image_transport message_filters nodelet rosconsole roscpp)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)

# Nodelet library
add_library(image_view src/nodelets/image_nodelet.cpp src/nodelets/disparity_nodelet.cpp)
target_link_libraries(image_view ${catkin_LIBRARIES}
                                 ${OpenCV_LIBRARIES}
                                 ${Boost_LIBRARIES}
)
install(TARGETS image_view
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Image viewers
add_executable(image_view_exe src/nodes/image_view.cpp)
SET_TARGET_PROPERTIES(image_view_exe PROPERTIES OUTPUT_NAME image_view)
target_link_libraries(image_view_exe ${catkin_LIBRARIES}
                                     ${OpenCV_LIBRARIES}
)

add_executable(disparity_view src/nodes/disparity_view.cpp)
target_link_libraries(disparity_view ${catkin_LIBRARIES}
                                     ${OpenCV_LIBRARIES}
)

add_executable(stereo_view src/nodes/stereo_view.cpp)
target_link_libraries(stereo_view ${Boost_LIBRARIES}
                                  ${catkin_LIBRARIES}
                                  ${OpenCV_LIBRARIES}
)


# Extra tools
add_executable(extract_images src/nodes/extract_images.cpp)
target_link_libraries(extract_images ${catkin_LIBRARIES}
                                     ${OpenCV_LIBRARIES}
)

add_executable(image_saver src/nodes/image_saver.cpp)
target_link_libraries(image_saver ${catkin_LIBRARIES}
                                  ${OpenCV_LIBRARIES}
)

add_executable(video_recorder src/nodes/video_recorder.cpp)
target_link_libraries(video_recorder ${catkin_LIBRARIES}
                                     ${OpenCV_LIBRARIES}
)

install(TARGETS disparity_view extract_images image_saver video_recorder image_view_exe stereo_view
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

project(image_view)
catkin_project(image_view)

find_package(Boost COMPONENTS signals)
find_package(GTK2)
find_package(OpenCV)
find_package(catkin REQUIRED camera_calibration_parsers cv_bridge image_transport rosconsole roscpp)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${camera_calibration_parsers_INCLUDE_DIRS}
                           ${cv_bridge_INCLUDE_DIRS}
                           ${GTK2_INCLUDE_DIRS}
                           ${image_transport_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)
add_definitions(-DHAVE_GTK)


# Nodelet library
if (FALSE)
rosbuild_add_library(image_view src/nodelets/image_nodelet.cpp src/nodelets/disparity_nodelet.cpp src/nodelets/window_thread.cpp)
target_link_libraries(image_view ${GTK_LIBRARIES}
                                 ${OpenCV_LIBRARIES}
)
install(TARGETS image_view
        DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION}
)

# Image viewers
add_executable(image_view_exe src/nodes/image_view.cpp)
SET_TARGET_PROPERTIES(image_view_exe PROPERTIES OUTPUT_NAME image_view)
target_link_libraries(image_view_exe
                      ${OpenCV_LIBRARIES}
)
endif()

if (FALSE)
add_executable(disparity_view src/nodes/disparity_view.cpp)
target_link_libraries(disparity_view
                      ${OpenCV_LIBRARIES}
)
endif()

add_executable(stereo_view src/nodes/stereo_view.cpp)
target_link_libraries(stereo_view ${Boost_LIBRARIES}
                                  ${cv_bridge_LIBRARIES}
                                  ${GTK_LIBRARIES}
                                  ${image_transport_LIBRARIES}
                                  ${OpenCV_LIBRARIES}
                                  ${rosconsole_LIBRARIES}
                                  ${roscpp_LIBRARIES}
)


# Extra tools
add_executable(extract_images src/nodes/extract_images.cpp)
target_link_libraries(extract_images ${cv_bridge_LIBRARIES}
                                     ${image_transport_LIBRARIES}
                                     ${OpenCV_LIBRARIES}
                                     ${rosconsole_LIBRARIES}
                                     ${roscpp_LIBRARIES}
)

add_executable(image_saver src/nodes/image_saver.cpp)
target_link_libraries(image_saver ${camera_calibration_parsers_LIBRARIES}
                                  ${cv_bridge_LIBRARIES}
                                  ${image_transport_LIBRARIES}
                                  ${OpenCV_LIBRARIES}
                                  ${rosconsole_LIBRARIES}
)

#install(TARGETS image_view_exe disparity_view stereo_view extract_images image_saver
install(TARGETS stereo_view extract_images image_saver
        DESTINATION ${CATKIN_PROJECT_BIN_DESTINATION}
)

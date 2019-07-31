cmake_minimum_required(VERSION 2.8.11)
project(FISHEYE_STEREO)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra")

find_package(OpenCV 3.4 EXACT REQUIRED PATHS /usr/local/)
include_directories($(OpenCV_INCLUDE_DIRS))

add_executable(calibrate calibrate.cpp)
target_link_libraries(calibrate ${OpenCV_LIBS} "-lpopt")
cmake_minimum_required(VERSION 3.5)
project(camera_calibration)

find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

# add tests
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()

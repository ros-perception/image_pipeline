cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()

# Unit test of calibrator.py
rosbuild_add_pyunit(test/directed.py)

# Unit test of the approximate synchronizer
rosbuild_add_pyunit(test/testapproxsync.py)

# Tests simple calibration dataset
rosbuild_download_test_data(http://pr.willowgarage.com/data/camera_calibration/camera_calibration.tar.gz camera_calibration.tar.gz 6da43ea314640a4c15dd7a90cbc3aee0)

rosbuild_add_pyunit(test/multiple_boards.py)

# Tests multiple checkerboards
rosbuild_download_test_data(http://pr.willowgarage.com/data/camera_calibration/multi_board_calibration.tar.gz multi_board_calibration.tar.gz ddc0f69582d140e33f9d3bfb681956bb)

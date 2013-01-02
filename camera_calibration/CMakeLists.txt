cmake_minimum_required(VERSION 2.8)
project(camera_calibration)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

# Unit test of calibrator.py
catkin_add_nosetests(test/directed.py)

# Unit test of the approximate synchronizer
catkin_add_nosetests(test/testapproxsync.py)

# Tests simple calibration dataset
download_test_data(http://pr.willowgarage.com/data/camera_calibration/camera_calibration.tar.gz camera_calibration.tar.gz 6da43ea314640a4c15dd7a90cbc3aee0)

# Tests multiple checkerboards
download_test_data(http://pr.willowgarage.com/data/camera_calibration/multi_board_calibration.tar.gz multi_board_calibration.tar.gz ddc0f69582d140e33f9d3bfb681956bb)
catkin_add_nosetests(test/multiple_boards.py)


install(PROGRAMS nodes/camera_calibrate_from_disk.py 
                 nodes/cameracalibrator.py
                 nodes/cameracheck.py
                 nodes/camera_hammer.py
                 scripts/tarfile_calibration.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

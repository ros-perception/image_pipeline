project(camera_calibration)
catkin_project(camera_calibration)

# Unit test of calibrator.py
rosbuild_add_pyunit(test/directed.py)

# Unit test of the approximate synchronizer
rosbuild_add_pyunit(test/testapproxsync.py)

# Tests simple calibration dataset
rosbuild_download_test_data(http://pr.willowgarage.com/data/camera_calibration/camera_calibration.tar.gz camera_calibration.tar.gz 6da43ea314640a4c15dd7a90cbc3aee0)

rosbuild_add_pyunit(test/multiple_boards.py)

# Tests multiple checkerboards
rosbuild_download_test_data(http://pr.willowgarage.com/data/camera_calibration/multi_board_calibration.tar.gz multi_board_calibration.tar.gz ddc0f69582d140e33f9d3bfb681956bb)

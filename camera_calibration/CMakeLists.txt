cmake_minimum_required(VERSION 2.8)
project(camera_calibration)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

if(CATKIN_ENABLE_TESTING)
  # Unit test of calibrator.py
  catkin_add_nosetests(test/directed.py)

  # Tests simple calibration dataset
  catkin_download_test_data(camera_calibration.tar.gz http://download.ros.org/data/camera_calibration/camera_calibration.tar.gz
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 6da43ea314640a4c15dd7a90cbc3aee0
  )

  # Tests multiple checkerboards
  catkin_download_test_data(multi_board_calibration.tar.gz http://download.ros.org/data/camera_calibration/multi_board_calibration.tar.gz
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 ddc0f69582d140e33f9d3bfb681956bb
  )
  catkin_add_nosetests(test/multiple_boards.py)
endif()

catkin_install_python(PROGRAMS nodes/cameracalibrator.py
  nodes/cameracheck.py
  scripts/tarfile_calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

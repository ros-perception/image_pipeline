^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package camera_calibration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

6.0.1 (2024-07-22)
------------------
* Change camera info message to lower case (`#1005 <https://github.com/ros-perception/image_pipeline/issues/1005>`_)
  Change camera info message to lower case since message type had been
  change in rolling and humble.
  [](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/CameraInfo.msg)
* Formatting calib code before refactoring (`#999 <https://github.com/ros-perception/image_pipeline/issues/999>`_)
  As discussed in `#975 <https://github.com/ros-perception/image_pipeline/issues/975>`_ and `#973 <https://github.com/ros-perception/image_pipeline/issues/973>`_
  doing the linting first.
  using style from
  [here](https://github.com/ament/ament_lint/blob/rolling/ament_pycodestyle/ament_pycodestyle/configuration/ament_pycodestyle.ini)
* Added stereo calibration using charuco board (`#976 <https://github.com/ros-perception/image_pipeline/issues/976>`_)
  From `#972 <https://github.com/ros-perception/image_pipeline/issues/972>`_
  Doing this first for rolling.
  This was a TODO in the repository, opening this PR to add this feature.
  - The main issue why this wasn't possible imo is the way `mk_obj_points`
  works. I'm using the inbuilt opencv function to get the points there.
  - The other is a condition when aruco markers are detected they are
  added as good points, This is fine in case of mono but in stereo these
  have to be the same number as the object points to find matches although
  this should be possible with aruco.
* Contributors: Myron Rodrigues, SFhmichael

6.0.0 (2024-05-27)
------------------
* fix: cv2.aruco.interpolateCornersCharuco is deprecated (`#979 <https://github.com/ros-perception/image_pipeline/issues/979>`_)
  There has been API Changes in the newer releases of opencv2 (from
  4.8.0). The PR addresses this by supporting both the old and new APIs.
  updated Syntax
  ```
  charucodetector = cv2.aruco.CharucoDetector(board)
  charuco_corners, charuco_ids, marker_corners, marker_ids = charucodetector.detectBoard(image)
  ```
  before 4.8.0
  ```
  marker_corners, marker_ids, rejectedImgPoints = cv2.aruco.detectMarkers( image, dictionary)
  retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco( marker_corners, marker_ids, image, board)
  ```
  See the changed examples in the main opencv2 repo:
  https://github.com/opencv/opencv/blob/f9a59f2592993d3dcc080e495f4f5e02dd8ec7ef/samples/python/calibrate.py#L110
* Update for compatibility with image_pipeline 4.1.0 (`#968 <https://github.com/ros-perception/image_pipeline/issues/968>`_)
  This is a PR to fix:
  - `#966 <https://github.com/ros-perception/image_pipeline/issues/966>`_
  As noted in `#966 <https://github.com/ros-perception/image_pipeline/issues/966>`_, as of writing image_pipeline [4.1.0 has been
  released](https://github.com/ros-perception/vision_opencv/releases/tag/4.1.0),
  is updated on
  [index.ros.org](https://index.ros.org/p/image_geometry/github-ros-perception-vision_opencv/#rolling),
  but it has not yet been migrated to
  [packages.ros.org](http://packages.ros.org/ros2/ubuntu/dists/noble/main/binary-amd64/Packages).
  As such `camera_calibration` will also require the source of
  [image_pipeline
  4.1.0](https://github.com/ros-perception/vision_opencv/releases/tag/4.1.0)
  or higher to successfully build.
  I tested to ensure successful build with colcon build & colcon test.
  Note that colcon test has the following warning that is out of scope of
  this PR:
  ```
  =============================== warnings summary ===============================
  src/camera_calibration/calibrator.py:47
  Warning: The distutils package is deprecated and slated for removal in Python 3.12. Use setuptools or check PEP 632 for potential alternatives
  ```
  Please let me know if there are any questions, concerns, or requested
  changes.
* replace disutils with python3-semver (`#970 <https://github.com/ros-perception/image_pipeline/issues/970>`_)
  Fix for
  - `#969 <https://github.com/ros-perception/image_pipeline/issues/969>`_
  I added a dependency for `python3-semver` to replace version parsing
  with `disutils`.
  Please let me know if you have any questions, concerns, or additional
  requested changes.
* Contributors: Földi Tamás, Scott Monaghan

5.0.1 (2024-03-26)
------------------
* Fix spelling error for cv2.aruco.DICT from 6x6_50 to 7x7_1000 (`#961 <https://github.com/ros-perception/image_pipeline/issues/961>`_)
  There was mismatch of capitalisation of "X" for OpenCV
  cv2.aruco.DICT_n**X**n\_ in camera_calibration package for dicts 6x6_50
  to 7x7_1000
  Co-authored-by: Vishal Balaji <vishal.balaji@schanzer-racing.de>
* unified changelog, add missing image, deduplicate tutorials (`#938 <https://github.com/ros-perception/image_pipeline/issues/938>`_)
  Last bit of documentation updates - putting together a single changelog
  summary for the whole release (rather than scattering among packages).
  Unified the camera_info tutorial so it isn't duplicated. Added a missing
  image from image_rotate (was on local disk, but hadn't committed it)
* migrate camera_calibration documentation (`#937 <https://github.com/ros-perception/image_pipeline/issues/937>`_)
* install tarfile_calibration (`#923 <https://github.com/ros-perception/image_pipeline/issues/923>`_)
  otherwise, it can't be run easily
* calibration: better warnings around board configuration `#713 <https://github.com/ros-perception/image_pipeline/issues/713>`_ (`#724 <https://github.com/ros-perception/image_pipeline/issues/724>`_)
* Contributors: Michael Ferguson, Vishal Balaji, jonathanTIE

5.0.0 (2024-01-24)
------------------
* ROS 2: Added more aruco dicts, fixed aruco linerror bug (`#873 <https://github.com/ros-perception/image_pipeline/issues/873>`_)
  Related with this PR in ROS 1
  https://github.com/ros-perception/image_pipeline/pull/795
* ROS 2: Fixing thrown Exception in camerachecker.py (`#871 <https://github.com/ros-perception/image_pipeline/issues/871>`_)
  Related with this PR in ROS 1
  https://github.com/ros-perception/image_pipeline/pull/812
* add myself as a maintainer (`#846 <https://github.com/ros-perception/image_pipeline/issues/846>`_)
* fix threading shutdown
* use correct synchronous service call
* use remap rules instead of parameters for services
* remove duplicated definition of on_model_change
* fix service check
* remove commented code
* Fix QoS incompatibility camera_calibration ROS2
* perform calibration in another thread
* Contributors: Alejandro Hernández Cordero, Christian Rauch, Kenji Brameld, Michael Ferguson, Michal Wojcik

3.0.1 (2022-12-04)
------------------
* add python3-opencv to camera calibration dependency
* port changes from `#755 <https://github.com/ros-perception/image_pipeline/issues/755>`_ to rolling branch
* Contributors: Kenji Brameld

3.0.0 (2022-04-29)
------------------
* Some small fixes noticed while reviewing.
* fix premature camera model change in camera_calibration
* Fix shebang lines for noetic python3
* Update fisheye distortion model definition
* Fix calibration yaml formatting (`#580 <https://github.com/ros-perception/image_pipeline/issues/580>`_) (`#585 <https://github.com/ros-perception/image_pipeline/issues/585>`_)
* updated linear_error function to handle partial board views (`#561 <https://github.com/ros-perception/image_pipeline/issues/561>`_)
* Fix missing detected checkerboard points (`#558 <https://github.com/ros-perception/image_pipeline/issues/558>`_)
* ChArUco board, Noetic (`#549 <https://github.com/ros-perception/image_pipeline/issues/549>`_)
* fix `#503 <https://github.com/ros-perception/image_pipeline/issues/503>`_: (`#545 <https://github.com/ros-perception/image_pipeline/issues/545>`_)
* Minimal Noetic (`#530 <https://github.com/ros-perception/image_pipeline/issues/530>`_)
* Apply `#509 <https://github.com/ros-perception/image_pipeline/issues/509>`_ and `#526 <https://github.com/ros-perception/image_pipeline/issues/526>`_ to Noetic Branch (`#528 <https://github.com/ros-perception/image_pipeline/issues/528>`_)
* Add Fisheye calibration tool (`#440 <https://github.com/ros-perception/image_pipeline/issues/440>`_)
* camera_calibration: Improve YAML formatting, make config dumping methods static (`#438 <https://github.com/ros-perception/image_pipeline/issues/438>`_)
* camera_calibration: Fix all-zero distortion coeffs returned for a rational_polynomial model (`#433 <https://github.com/ros-perception/image_pipeline/issues/433>`_)
* Make sure 'calibrate' button works even if not receiving images anymore
* Add a comment
* Replace deque with a modified Queue, add --queue-size param
* Remove print statement
* Cosmetic changes
* Add max-chessboard-speed option to allow more accurate calibration of rolling shutter cameras.
* revert back
* added missing imports
* update pytest.ini
* fixes to pass tests
* rebase change
* implemented fisheye mono and stereo calibration based on the melodic branch
* trimmed whitespace at line endings
* Update camera_calibration setup.cfg to use underscores (`#688 <https://github.com/ros-perception/image_pipeline/issues/688>`_)
* Add maintainer (`#667 <https://github.com/ros-perception/image_pipeline/issues/667>`_)
* Fixed crash when rosargs are given (`#597 <https://github.com/ros-perception/image_pipeline/issues/597>`_)
* Contributors: Chris Lalancette, David Torres Ocaña, DavidTorresOcana, Gabor Soros, Jacob Perron, John Stechschulte, Joshua Whitley, Martin Valgur, Matthijs den Toom, Michael Carroll, Patrick Musau, Photon, Spiros Evangelatos, Victor Dubois, jaiveersinghNV, soeroesg

2.2.1 (2020-08-27)
------------------
* remove email blasts from steve macenski (`#596 <https://github.com/ros-perception/image_pipeline/issues/596>`_)
* Add pytest.ini to fix warning (`#584 <https://github.com/ros-perception/image_pipeline/issues/584>`_)
  Fixes the following warning:
  Warning: The 'junit_family' default value will change to 'xunit2' in pytest 6.0.
  Add 'junit_family=xunit1' to your pytest.ini file to keep the current format in future versions of pytest and silence this warning.
* [Foxy] Use ament_auto Macros (`#573 <https://github.com/ros-perception/image_pipeline/issues/573>`_)
* Contributors: Jacob Perron, Joshua Whitley, Steve Macenski

2.2.0 (2020-07-27)
------------------
* Removed basestring (no longer exists in new python 3 version). (`#554 <https://github.com/ros-perception/image_pipeline/issues/554>`_)
  Fixes `#551 <https://github.com/ros-perception/image_pipeline/issues/551>`_
* Initial ROS2 commit.
* Contributors: Michael Carroll, PfeifferMicha

1.12.23 (2018-05-10)
--------------------
* camera_checker: Ensure cols + rows are in correct order (`#319 <https://github.com/ros-perception/image_pipeline/issues/319>`_)
  Without this commit, specifying a smaller column than row size lead to
  huge reported errors:
  ```
  $ rosrun camera_calibration cameracheck.py --size 6x7 --square 0.0495
  Linearity RMS Error: 13.545 Pixels      Reprojection RMS Error: 22.766 Pixels
  $ rosrun camera_calibration cameracheck.py --size 7x6 --square 0.0495
  Linearity RMS Error: 0.092 Pixels      Reprojection RMS Error: 0.083 Pixels
  ```
  This commit switches columns and rows around if necessary.
* Contributors: Martin Günther

1.12.22 (2017-12-08)
--------------------
* Changed flags CV_LOAD_IMAGE_COLOR by IMREAD_COLOR to adapt to Opencv3. (`#252 <https://github.com/ros-perception/image_pipeline/issues/252>`_)
* Fixed stereo calibration problem with chessboard with the same number of rows and cols by rotating the corners to same direction.
* Contributors: jbosch

1.12.21 (2017-11-05)
--------------------
* re-add the calibration nodes but now using the Python modules.
  Fixes `#298 <https://github.com/ros-perception/image_pipeline/issues/298>`_
* Move nodes to Python module.
* Contributors: Vincent Rabaud

1.12.20 (2017-04-30)
--------------------
* properly save bytes buffer as such
  This is useful for Python 3 and fixes `#256 <https://github.com/ros-perception/image_pipeline/issues/256>`_.
* Get tests slightly looser.
  OpenCV 3.2 gives slightly different results apparently.
* Use floor division where necessary. (`#247 <https://github.com/ros-perception/image_pipeline/issues/247>`_)
* Fix and Improve Camera Calibration Checker Node (`#254 <https://github.com/ros-perception/image_pipeline/issues/254>`_)
  * Fix according to calibrator.py API
  * Add approximate to cameracheck
* Force first corner off chessboard to be uppler left.
  Fixes `#140 <https://github.com/ros-perception/image_pipeline/issues/140>`_
* fix doc jobs
  This is a proper fix for `#233 <https://github.com/ros-perception/image_pipeline/issues/233>`_
* During stereo calibration check that the number of corners detected in the left and right images are the same. This fixes `ros-perception/image_pipeline#225 <https://github.com/ros-perception/image_pipeline/issues/225>`_
* Contributors: Leonard Gerard, Martin Peris, Vincent Rabaud, hgaiser

1.12.19 (2016-07-24)
--------------------
* Fix array check in camerachecky.py
  This closes `#205 <https://github.com/ros-perception/image_pipeline/issues/205>`_
* Contributors: Vincent Rabaud

1.12.18 (2016-07-12)
--------------------

1.12.17 (2016-07-11)
--------------------
* fix typo np -> numpy
* fix failing tests
* Contributors: Shingo Kitagawa, Vincent Rabaud

1.12.16 (2016-03-19)
--------------------
* clean OpenCV dependency in package.xml
* Contributors: Vincent Rabaud

1.12.15 (2016-01-17)
--------------------
* better 16 handling in mkgray
  This re-uses `#150 <https://github.com/ros-perception/image_pipeline/issues/150>`_ and therefore closes `#150 <https://github.com/ros-perception/image_pipeline/issues/150>`_
* fix OpenCV2 compatibility
* fix tests with OpenCV3
* [Calibrator]: add yaml file with calibration data in output
* Contributors: Vincent Rabaud, sambrose

1.12.14 (2015-07-22)
--------------------
* remove camera_hammer and install Python nodes properly
  camera_hammer was just a test for camera info, nothing to do with
  calibration. Plus the test was basic.
* Correct three errors that prevented the node to work properly.
* Contributors: Filippo Basso, Vincent Rabaud

1.12.13 (2015-04-06)
--------------------
* replace Queue by deque of fixed size for simplicity
  That is a potential fix for `#112 <https://github.com/ros-perception/image_pipeline/issues/112>`_
* Contributors: Vincent Rabaud

1.12.12 (2014-12-31)
--------------------
* try to improve `#112 <https://github.com/ros-perception/image_pipeline/issues/112>`_
* Contributors: Vincent Rabaud

1.12.11 (2014-10-26)
--------------------

1.12.10 (2014-09-28)
--------------------
* Update calibrator.py
  bugfix: stereo calibrator crashed after the signature of the method for the computation of the epipolar error changed but the function call was not updated
* Contributors: Volker Grabe

1.12.9 (2014-09-21)
-------------------
* fix bad Python
* only analyze the latest image
  fixes `#97 <https://github.com/ros-perception/image_pipeline/issues/97>`_
* flips width and height during resize to give correct aspect ratio
* Contributors: Russell Toris, Vincent Rabaud

1.12.8 (2014-08-19)
-------------------
* install scripts in the local bin (they are now rosrun-able again)
  fixes `#93 <https://github.com/ros-perception/image_pipeline/issues/93>`_
* fix default Constructor for OpenCV flags
  this does not change anything in practice as the flag is set by the node.
  It just fixes the test.
* Contributors: Vincent Rabaud

1.12.6 (2014-07-27)
-------------------
* make sure the GUI is started in its processing thread and fix a typo
  This fully fixes `#85 <https://github.com/ros-perception/image_pipeline/issues/85>`_
* fix bad call to save an image
* have display be in its own thread
  that could be a fix for `#85 <https://github.com/ros-perception/image_pipeline/issues/85>`_
* fix bad usage of Numpy
  fixes `#89 <https://github.com/ros-perception/image_pipeline/issues/89>`_
* fix asymmetric circle calibration
  fixes `#35 <https://github.com/ros-perception/image_pipeline/issues/35>`_
* add more tests
* improve unittests to include all patterns
* install Python scripts properly
  and fixes `#86 <https://github.com/ros-perception/image_pipeline/issues/86>`_
* fix typo that leads to segfault
  fixes `#84 <https://github.com/ros-perception/image_pipeline/issues/84>`_
* also print self.report() on calibrate ... allows to use the params without having to commit them (e.g. for extrensic calibration between to cameras not used as stereo pair)
* fixes `#76 <https://github.com/ros-perception/image_pipeline/issues/76>`_
  Move Python approximate time synchronizer to ros_comm
* remove all trace of cv in Python (use cv2)
* remove deprecated file (as mentioned in its help)
* fixes `#25 <https://github.com/ros-perception/image_pipeline/issues/25>`_
  This is just removing deprecated options that were around since diamondback
* fixes `#74 <https://github.com/ros-perception/image_pipeline/issues/74>`_
  calibrator.py is now using the cv2 only API when using cv_bridge.
  The API got changed too but it seems to only be used internally.
* Contributors: Vincent Rabaud, ahb

1.12.5 (2014-05-11)
-------------------
* Fix `#68 <https://github.com/ros-perception/image_pipeline/issues/68>`_: StringIO issues in calibrator.py
* fix architecture independent
* Contributors: Miquel Massot, Vincent Rabaud

1.12.4 (2014-04-28)
-------------------

1.12.3 (2014-04-12)
-------------------
* camera_calibration: Fix Python import order
* Contributors: Scott K Logan

1.12.2 (2014-04-08)
-------------------
* Fixes a typo on stereo camera info service calls
  Script works after correcting the call names.
* Contributors: JoonasMelin

1.11.4 (2013-11-23 13:10:55 +0100)
----------------------------------
- add visualization during calibration and several calibration flags (#48)

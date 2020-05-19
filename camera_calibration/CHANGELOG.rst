1.15.0 (2020-05-18)
-------------------
* Fix import path on camera_calibrator.py (`#509 <https://github.com/ros-perception/image_pipeline/issues/509>`_)
* Fixes `#501 <https://github.com/ros-perception/image_pipeline/issues/501>`_: self.size is set before dumping calibration parameters in calibrator.py do_calibration(self, dump) (`#502 <https://github.com/ros-perception/image_pipeline/issues/502>`_)
  Not sure how this one made it through. Thanks for the fix!
* Contributors: Gonçalo Camelo Neves Pereira, Stewart Jamieson

1.14.0 (2020-01-12)
-------------------
* Add Fisheye calibration tool (`#440 <https://github.com/ros-perception/image_pipeline/issues/440>`_)
  * Add Fisheye calibration tool
  * Restore camera_calib files permisions
  * Upgrades to calibrator tool for multi model calibration
  * Solve fisheye balance selection
  * Add fisheye calibration flags as user arguments
  * Add undistortion of points for fisheye
  * cam_calib: Style formating
* camera_calibration: Improve YAML formatting, make config dumping methods static (`#438 <https://github.com/ros-perception/image_pipeline/issues/438>`_)
  * Add `from __future_\_ import print_function`
  * Improve YAML formatting, make some methods static
  * Improves matrix formatting in YAML.
  * Reduced decimal figures for camera and projection matrix values from 8 to 5.
  * Making the methods static allows them to be used from elsewhere as well to dump calibration info.
* camera_calibration: Fix all-zero distortion coeffs returned for a rational_polynomial model (`#433 <https://github.com/ros-perception/image_pipeline/issues/433>`_)
  * Fix empty distortion coeffs returned for a rational_polynomial model
  * Remove the redundant distCoeffs parameter from cv2.calibrateCamera()
  * Set the shape of distortion_coefficients correctly in YAML output
* Merge pull request `#437 <https://github.com/ros-perception/image_pipeline/issues/437>`_ from valgur/enable-calibration-with-empty-queue
  camera_calibration: Make sure 'calibrate' button works even if not receiving images anymore
* Make sure 'calibrate' button works even if not receiving images anymore
* Merge pull request `#432 <https://github.com/ros-perception/image_pipeline/issues/432>`_ from valgur/melodic
  camera_calibration: Fix excessive CPU usage due to queue synchronization
* Replace deque with a modified Queue, add --queue-size param
  Base fork on upstream melodic instead of indigo
* Contributors: David Torres Ocaña, Joshua Whitley, Martin Valgur, Tim Übelhör

1.13.0 (2019-06-12)
-------------------
* Merge pull request `#356 <https://github.com/ros-perception/image_pipeline/issues/356>`_ from sevangelatos/feature/calibrator_rolling_shutter
* Add max-chessboard-speed option to allow more accurate calibration of rolling shutter cameras.
* Merge pull request `#334 <https://github.com/ros-perception/image_pipeline/issues/334>`_ from Fruchtzwerg94/patch-2
  Scale pixels down from 16 to 8 bits instead of just clipping
* Merge pull request `#340 <https://github.com/ros-perception/image_pipeline/issues/340>`_ from k-okada/286
  use waitKey(0) instead of while loop
* Merge pull request `#395 <https://github.com/ros-perception/image_pipeline/issues/395>`_ from ros-perception/steve_maintain
* adding autonomoustuff mainainer
* adding stevemacenski as maintainer to get emails
* Scale pixels down from 16 to 8 bits instead of just clipping
  Clipping 16 bit pixels just to 8 bit pixels leads to white images if the original image uses the full range of the 16 bits. Instead the pixel should be scaled down to 8 bits.
* Contributors: Joshua Whitley, Kei Okada, Philipp, Spiros Evangelatos, Yoshito Okada, stevemacenski

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

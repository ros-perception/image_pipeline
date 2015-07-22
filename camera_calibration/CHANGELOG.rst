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

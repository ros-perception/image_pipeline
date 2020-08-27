2.1.1 (2020-08-27)
------------------
* Disable "Publish Color!" debug_info (`#578 <https://github.com/ros-perception/image_pipeline/issues/578>`_)
* [Dashing] Use ament_auto Macros (`#573 <https://github.com/ros-perception/image_pipeline/issues/573>`_) (`#575 <https://github.com/ros-perception/image_pipeline/issues/575>`_)
* Contributors: Dereck Wonnacott, Joshua Whitley

2.1.0 (2020-07-27)
------------------
* Fixing flake8 error in image_proc.
* Opencv 3 compatibility (`#564 <https://github.com/ros-perception/image_pipeline/issues/564>`_) (`#565 <https://github.com/ros-perception/image_pipeline/issues/565>`_)
  * Remove GTK from image_view.
  * Reinstate OpenCV 3 compatibility.
  Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* Removed namespaces as parameters and added launch file example in image_proc (`#519 <https://github.com/ros-perception/image_pipeline/issues/519>`_)
* Commented out getNumSubscribers on resize.cpp (`#523 <https://github.com/ros-perception/image_pipeline/issues/523>`_)
* Switched default interpolation in recitfy.cpp to mode 1 (`#522 <https://github.com/ros-perception/image_pipeline/issues/522>`_)
* Build image_proc as shared library and export (`#513 <https://github.com/ros-perception/image_pipeline/issues/513>`_)
* Add image_proc launch file (`#492 <https://github.com/ros-perception/image_pipeline/issues/492>`_)
  * Add image_proc launch file
  * Install launch file
* [image_proc] Install include directory and export (`#485 <https://github.com/ros-perception/image_pipeline/issues/485>`_)
* Port image processor library to ROS 2 (`#484 <https://github.com/ros-perception/image_pipeline/issues/484>`_)
* Merge pull request `#459 <https://github.com/ros-perception/image_pipeline/issues/459>`_ from klintan/image-proc-resize-crop
  ROS2: Image proc non zero crop
* fixed linting issues
* ROS2: Lint/Uncrustify image_proc (`#474 <https://github.com/ros-perception/image_pipeline/issues/474>`_)
* cleanup any last reference to nodelets & register image publisher as a component (`#473 <https://github.com/ros-perception/image_pipeline/issues/473>`_)
* Merge pull request `#471 <https://github.com/ros-perception/image_pipeline/issues/471>`_ from ros-perception/crop_d
  ROS2 port of crop decimate in image proc
* Merge pull request `#470 <https://github.com/ros-perception/image_pipeline/issues/470>`_ from ros-perception/crop_ros2
* fix linter and adding components dependencies to packages requiring them
* ROS2: Added resize component (`#465 <https://github.com/ros-perception/image_pipeline/issues/465>`_)
* Merge pull request `#450 <https://github.com/ros-perception/image_pipeline/issues/450>`_ from ros-perception/revert-443-dashing-launch-file
* Revert "ROS2 Image proc refactoring using components and added launch file"
* Merge pull request `#443 <https://github.com/ros-perception/image_pipeline/issues/443>`_ from klintan/dashing-launch-file
  ROS2 Image proc refactoring using components and added launch file
* Merge pull request `#426 <https://github.com/ros-perception/image_pipeline/issues/426>`_ from klintan/image-proc-dashing
  Dashing: Image_proc with old PR comments fixed
* Initial ROS2 commit.
* Contributors: Andreas Klintberg, Jacob Perron, Joshua Whitley, Michael Carroll, Steven Macenski, Yoshito Okada, caelinsutch, stevemacenski

1.12.23 (2018-05-10)
--------------------

1.12.22 (2017-12-08)
--------------------
* Merge pull request `#311 <https://github.com/ros-perception/image_pipeline/issues/311>`_ from knorth55/revert-299
  Revert "Fix image_resize nodelet (`#299 <https://github.com/ros-perception/image_pipeline/issues/299>`_)"
  This reverts commit 32e19697ebce47101b063c6a02b95dfa2c5dbc52.
* Contributors: Shingo Kitagawa, Tully Foote

1.12.21 (2017-11-05)
--------------------
* Fix image_resize nodelet (`#299 <https://github.com/ros-perception/image_pipeline/issues/299>`_)
  Update interpolation types
  Add arguments to enable disable each nodelet
  Add default arguments for image_resize and image_rect
  Use toCVShare instead of toCVCopy
  Include image_resize in image_proc
* Updated fix for traits change. (`#303 <https://github.com/ros-perception/image_pipeline/issues/303>`_)
* Fix C++11 compilation
  This fixes `#292 <https://github.com/ros-perception/image_pipeline/issues/292>`_ and `#291 <https://github.com/ros-perception/image_pipeline/issues/291>`_
* [image_proc][crop_decimate] support changing target image frame_id (`#276 <https://github.com/ros-perception/image_pipeline/issues/276>`_)
* Contributors: Furushchev, Mike Purvis, Vincent Rabaud, bikramak

1.12.20 (2017-04-30)
--------------------
* Add nodelet to resize image and camera_info (`#273 <https://github.com/ros-perception/image_pipeline/issues/273>`_)
  * Add nodelet to resize image and camera_info
  * Depends on nodelet_topic_tools
  * Use recursive_mutex for mutex guard for dynamic reconfiguring
* Fix nodelet name: crop_nonZero ->  crop_non_zero (`#269 <https://github.com/ros-perception/image_pipeline/issues/269>`_)
  Fix https://github.com/ros-perception/image_pipeline/issues/217
* Fix permission of executable files unexpectedly (`#260 <https://github.com/ros-perception/image_pipeline/issues/260>`_)
* address gcc6 build error
  With gcc6, compiling fails with `stdlib.h: No such file or directory`,
  as including '-isystem /usr/include' breaks with gcc6, cf.,
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70129.
  This commit addresses this issue for this package in the same way
  it was addressed in various other ROS packages. A list of related
  commits and pull requests is at:
  https://github.com/ros/rosdistro/issues/12783
  Signed-off-by: Lukas Bulwahn <lukas.bulwahn@oss.bmw-carit.de>
* Contributors: Kentaro Wada, Lukas Bulwahn

1.12.19 (2016-07-24)
--------------------

1.12.18 (2016-07-12)
--------------------

1.12.17 (2016-07-11)
--------------------

1.12.16 (2016-03-19)
--------------------
* clean OpenCV dependency in package.xml
* issue `#180 <https://github.com/ros-perception/image_pipeline/issues/180>`_ Check if all distortion coefficients are zero.
  Test with:
  rostest --reuse-master --text image_proc test_rectify.xml
  Can also test interactively with vimjay image_rect.launch, which brings up an rqt gui and camera info distortion coefficients can be dynamically reconfigured.
* Add a feature to crop the largest valid (non zero) area
  Remove unnecessary headers
  change a filename to fit for the ROS convention
* Contributors: Kenta Yonekura, Lucas Walter, Vincent Rabaud

1.12.15 (2016-01-17)
--------------------
* simplify OpenCV3 conversion
* Contributors: Vincent Rabaud

1.12.14 (2015-07-22)
--------------------

1.12.13 (2015-04-06)
--------------------
* fix dependencies
* Contributors: Vincent Rabaud

1.12.12 (2014-12-31)
--------------------

1.12.11 (2014-10-26)
--------------------

1.12.10 (2014-09-28)
--------------------

1.12.9 (2014-09-21)
-------------------
* get code to compile with OpenCV3
  fixes `#96 <https://github.com/ros-perception/image_pipeline/issues/96>`_
* Contributors: Vincent Rabaud

1.12.8 (2014-08-19)
-------------------

1.12.6 (2014-07-27)
-------------------

1.12.4 (2014-04-28)
-------------------

1.12.3 (2014-04-12)
-------------------

1.12.2 (2014-04-08)
-------------------

1.12.1 (2014-04-06)
-------------------
* get proper opencv dependency
* Contributors: Vincent Rabaud

1.11.7 (2014-03-28)
-------------------

1.11.6 (2014-01-29 00:38:55 +0100)
----------------------------------
- fix bad OpenCV linkage (#53)

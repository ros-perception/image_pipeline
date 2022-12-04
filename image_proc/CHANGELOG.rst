2.3.0 (2022-12-04)
------------------
* Add conversion from YUV422-YUY2
* Remove unnecessary find_package
  tracetools_image_pipeline included in package.xml and
  fetched by ament_auto_find_build_dependencies().
* Deal with uncrustify and cpplint
* LTTng instrument image_proc::RectifyNode and image_proc::ResizeNode
* bring over ros1 fix for missing roi resize
* Add maintainer (`#667 <https://github.com/ros-perception/image_pipeline/issues/667>`_)
* Fix build with later versions of OpenCV 3
  This first regressed in 2b17a38a5e3d4aef9c6a51c2de10d7396c521648.
  Support for OpenCV 3.2 was re-added in
  2e0c6d42cb650534e4aeea586482030e5c0d46c8. This fixes the build with
  OpenCV 3.3 and newer.
* Refactor image_proc and stereo_image_proc launch files (`#583 <https://github.com/ros-perception/image_pipeline/issues/583>`_)
  * Allow passing container name to image_proc launch file
  If a container name is provided, then load the image_proc nodes
  into that container. Otherwise, launch a container to load the nodes into.
  * Include the image_proc launch file in the stereo_image_proc launch file
  This resolves a TODO, making the launch file have similar behavior as the version from ROS 1.
  Also expose a new launch argument for optionally providing a container (similar to image_proc's launch file).
  * Minor refactor to stereo_image_proc launch file
  * Fix lint errors
  Removing vestigial imports.
  * Rename namespace launch arguments
  * Make image_proc nodes optional
  Default to launching the image_proc nodes.
  * Remap topics from stereo nodes based on namespace arguments
* Contributors: Evan Flynn, Jacob Perron, Scott K Logan, Tillmann Falck, Víctor Mayoral Vilches

2.2.1 (2020-08-27)
------------------
* make crop_decimate work (`#593 <https://github.com/ros-perception/image_pipeline/issues/593>`_)
* remove email blasts from steve macenski (`#596 <https://github.com/ros-perception/image_pipeline/issues/596>`_)
* Disable "Publish Color!" debug_info (`#577 <https://github.com/ros-perception/image_pipeline/issues/577>`_)
* [Foxy] Use ament_auto Macros (`#573 <https://github.com/ros-perception/image_pipeline/issues/573>`_)
* Contributors: Dereck Wonnacott, Joshua Whitley, Michael Ferguson, Steve Macenski

2.2.0 (2020-07-27)
------------------
* Replacing deprecated header includes with new HPP versions. (`#566 <https://github.com/ros-perception/image_pipeline/issues/566>`_)
* Opencv 3 compatibility (`#564 <https://github.com/ros-perception/image_pipeline/issues/564>`_)
  * Remove GTK from image_view.
  * Reinstate OpenCV 3 compatibility.
* Fix bad quotes in image_proc launch file (`#563 <https://github.com/ros-perception/image_pipeline/issues/563>`_)
  This fixes a flake8 error.
* Contributors: Chris Lalancette, Jacob Perron, Joshua Whitley

* Initial ROS2 commit.
* Contributors: Michael Carroll

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

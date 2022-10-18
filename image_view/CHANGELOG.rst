1.17.0 (2022-10-17)
-------------------
* Switch to hpp headers of pluginlib
* Switch to new boost/bind/bind.hpp
* Add support for floating point fps
* Contributors: Jochen Sprickerhof, Júnio Eduardo de Morais Aquino

1.16.0 (2021-11-12)
-------------------
* remove GTK3 dep.
* remove harfbuzz.
* Update image_view/CMakeLists.txt
  Co-authored-by: Joshua Whitley <josh.whitley@autoware.org>
* Update image_view/CMakeLists.txt
  Co-authored-by: Joshua Whitley <josh.whitley@autoware.org>
* Make GTK3 and harfbuzz optional
* Contributors: Sean Yen, seanyen

1.15.3 (2020-12-11)
-------------------
* remove email blasts from steve macenski (`#595 <https://github.com/ros-perception/image_pipeline/issues/595>`_)
* [image_view] Warn when filename_format is invalid (`#587 <https://github.com/ros-perception/image_pipeline/issues/587>`_)
* Contributors: Naoya Yamaguchi, Steve Macenski

1.15.2 (2020-05-19)
-------------------

1.15.1 (2020-05-18)
-------------------
* image_view: add missing dependency to gencfg header (`#531 <https://github.com/ros-perception/image_pipeline/issues/531>`_)
* Contributors: Atsushi Watanabe

1.15.0 (2020-05-14)
-------------------
* Python 3 compatibility (`#530 <https://github.com/ros-perception/image_pipeline/issues/530>`_)
* cmake_minimum_required to 3.0.2
* Adapted to OpenCV4
* import setup from setuptools instead of distutils-core
* Apply `#509 <https://github.com/ros-perception/image_pipeline/issues/509>`_ and `#526 <https://github.com/ros-perception/image_pipeline/issues/526>`_ to Noetic Branch (`#528 <https://github.com/ros-perception/image_pipeline/issues/528>`_)
* [image_view] Add dynamic reconfigure to image_nodelet.cpp in melodic (`#504 <https://github.com/ros-perception/image_pipeline/issues/504>`_)
* updated install locations for better portability. (`#500 <https://github.com/ros-perception/image_pipeline/issues/500>`_)
* Contributors: Joshua Whitley, Naoya Yamaguchi, Sean Yen

1.14.0 (2020-01-12)
-------------------
* Merge pull request `#481 <https://github.com/ros-perception/image_pipeline/issues/481>`_ from ros-perception/fix/reliably-close-image-view
* image_view: Making window close reliably shut down node.
* Removing image_view node and replacing with image_view that loads nodelet. (`#479 <https://github.com/ros-perception/image_pipeline/issues/479>`_)
* Fix build issue re: missing hb.h (`#458 <https://github.com/ros-perception/image_pipeline/issues/458>`_)
* Contributors: Joshua Whitley, Steven Macenski, Tim Übelhör, acxz

1.13.0 (2019-06-12)
-------------------
* Implemented extracting raw image data (`#329 <https://github.com/ros-perception/image_pipeline/issues/329>`_)
  Implementation of the raw image extraction if the file extension is .raw. This file extension is not supported by cv::imwrite so there is be no conflict. The raw files only containing the pixel data without any meta data which allows usage in MATLAB or other tools.
* Merge pull request `#375 <https://github.com/ros-perception/image_pipeline/issues/375>`_ from fizyr-forks/opencv4
* Fix OpenCV4 compatibility.
* Merge pull request `#337 <https://github.com/ros-perception/image_pipeline/issues/337>`_ from yoshito-okada/fix_image_view_nodelet
  Fix threading issue in image_view nodelet. Closes `#331 <https://github.com/ros-perception/image_pipeline/issues/331>`_.
* Merge pull request `#394 <https://github.com/ros-perception/image_pipeline/issues/394>`_ from angeltop/indigo
  image_view: video recorder fix for conversion of fps to ros::Duration
* Merge pull request `#379 <https://github.com/ros-perception/image_pipeline/issues/379>`_ from fizyr-forks/boost-1.69
  Fix boost 1.69 compatibility
* Merge pull request `#395 <https://github.com/ros-perception/image_pipeline/issues/395>`_ from ros-perception/steve_maintain
* adding stevemacenski as maintainer to get emails
* adding autonomoustuff mainainer
* Merge pull request `#343 <https://github.com/ros-perception/image_pipeline/issues/343>`_ from fkie-forks/work_around_opencv_highgui_bug
  Work around OpenCV highgui bug
  I had to remove the GTK workaround, since it creates symbol collisions between GTK2 and GTK3.
* Refresh GUI on image update as well
* Use WallTimer for backwards compatibility with ROS Indigo
* Switch to SteadyTimer as suggested in review
* Remove unused `signals` from find_package(Boost COMPONENTS ...)
  The signals library was not used at all, and it has been removed from
  boost 1.69. As a result, the package doesn't build anymore with boost
  1.69 without this change.
* While we're at it, work around the mutex assertion failure on exit
* Work around an OpenCV bug with GTK and threading
* add ThreadSageImage class to encapsilate mutex operation for image
* handle window in single thread
* Contributors: Hans Gaiser, Joshua Whitley, Maarten de Vries, Philipp, Timo Röhling, Yoshito Okada, angeltop, stevemacenski

1.12.23 (2018-05-10)
--------------------

1.12.22 (2017-12-08)
--------------------

1.12.21 (2017-11-05)
--------------------
* call namedWindow from same thread as imshow, need waitKay, now cvStartWindowThreads is null funciton on window_QT.h (`#279 <https://github.com/ros-perception/image_pipeline/issues/279>`_)
* Contributors: Kei Okada

1.12.20 (2017-04-30)
--------------------
* DisparityViewNodelet: fixed freeze (`#244 <https://github.com/ros-perception/image_pipeline/issues/244>`_)
* launch image view with a predefined window size (`#257 <https://github.com/ros-perception/image_pipeline/issues/257>`_)
* Remove python-opencv run_depend for image_view (`#270 <https://github.com/ros-perception/image_pipeline/issues/270>`_)
  The `python-opencv` dependency pulls in the system OpenCV v2.4 which is
  not required since the `image_view` package depends on `cv_bridge` which
  pulls in `opencv3` and `opencv3` provides the python library that
  `image_view` can use.
* Fix encoding error message (`#253 <https://github.com/ros-perception/image_pipeline/issues/253>`_)
  * Fix encoding error message
  * Update image_saver.cpp
  Allow compilation on older compilers
* Including stereo_msgs dep fixes `#248 <https://github.com/ros-perception/image_pipeline/issues/248>`_ (`#249 <https://github.com/ros-perception/image_pipeline/issues/249>`_)
* Add no gui mode to just visualize & publish with image_view (`#241 <https://github.com/ros-perception/image_pipeline/issues/241>`_)
* stere_view: fixed empty left, right, disparity windows with opencv3
* Apply value scaling to depth/float image with min/max image value
  If min/max image value is specified we just use it, and if not,
  - 32FC1: we assume depth image with meter metric, and 10[m] as the max range.
  - 16UC1: we assume depth image with milimeter metric, and 10 * 1000[mm] as the max range.
* Depends on cv_bridge 1.11.13 for CvtColorForDisplayOptions
  Close `#238 <https://github.com/ros-perception/image_pipeline/issues/238>`_
* fix doc jobs
  This is a proper fix for `#233 <https://github.com/ros-perception/image_pipeline/issues/233>`_
* address gcc6 build error
  With gcc6, compiling fails with `stdlib.h: No such file or directory`,
  as including '-isystem /usr/include' breaks with gcc6, cf.,
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70129.
  This commit addresses this issue for this package in the same way
  it was addressed in various other ROS packages. A list of related
  commits and pull requests is at:
  https://github.com/ros/rosdistro/issues/12783
  Signed-off-by: Lukas Bulwahn <lukas.bulwahn@oss.bmw-carit.de>
* Contributors: Christopher Wecht, Kartik Mohta, Kei Okada, Kentaro Wada, Lukas Bulwahn, Leonard Gerard, Vincent Rabaud, cwecht, mryellow

1.12.19 (2016-07-24)
--------------------
* Add colormap option in video_recorder
* Merge pull request `#203 <https://github.com/ros-perception/image_pipeline/issues/203>`_ from wkentaro/video-recorder-timestamp
  [image_view] Stamped video output filename for video recorder
* bump version requirement for cv_bridge dep
  Closes `#215 <https://github.com/ros-perception/image_pipeline/issues/215>`_
* Request for saving image with start/end two triggers
* Stamped video output filename
  - _filename:=output.avi _stamped_filename:=false -> output.avi
  - _filename:=_out.avi _stamped_filename:=true -> 1466299931.584632829_out.avi
  - _filename:=$HOME/.ros/.avi _stamped_filename:=true -> /home/ubuntu/.ros/1466299931.584632829.avi
* Revert max_depth_range to default value for cvtColorForDisplay
* Contributors: Kentaro Wada, Vincent Rabaud

1.12.18 (2016-07-12)
--------------------
* Use image_transport::Subscriber aside from ros::Subscriber
* Refactor: Remove subscription of camera_info in video_recorder
* Add colormap options for displaying image topic
* Use CvtColorForDisplayOptions for cvtColorForDisplay
* Contributors: Kentaro Wada, Vincent Rabaud

1.12.17 (2016-07-11)
--------------------
* Fix timestamp to get correct fps in video_recorder
* Get correct fps in video_recorder.cpp
* Do dynamic scaling for float images
* Contributors: Kentaro Wada

1.12.16 (2016-03-19)
--------------------
* Remove code for roslib on .cfg files
  Closes `#185 <https://github.com/ros-perception/image_pipeline/issues/185>`_
* add cv::waitKey for opencv3 installed from source to fix freezing issue
* when no image is saved, do not save camera info
  When the images are not recorded because "save_all_image" is false and "save_image_service" is false, the frame count should not be incremented and the camera info should not be written to disk.
* Add std_srvs to catkin find_package()
* Contributors: Jeremy Kerfs, Jochen Sprickerhof, Kentaro Wada, Krishneel

1.12.15 (2016-01-17)
--------------------
* simplify the OpenCV dependency
* [image_view] Configure do_dynamic_scaling param with dynamic_reconfigure
* [image_view] Scale 16UC1 depth image
* fix compilation
* Extract images which are synchronized with message_filters
* [image_view] Show full path when failed to save image
* [image_view] Enable to specify transport with arg
* [image_view] feedback: no need threading for callback
* [image_view/image_view] Make as a node
* Added sensor_msgs::Image conversion to cv::Mat from rqt_image_view in
  order to be able to create videos from kinect depth images (cv_bridge
  currently doesn't support 16UC1 image encoding).
  Code adapted from:
  https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_image_view/src/rqt_image_view/image_view.cpp
* simplify OpenCV3 conversion
* use the color conversion for display from cv_bridge
* Contributors: Carlos Costa, Kentaro Wada, Vincent Rabaud

1.12.14 (2015-07-22)
--------------------
* reduce the differences between OpenCV2 and 3
* do not build GUIs on Android
  This fixes `#137 <https://github.com/ros-perception/image_pipeline/issues/137>`_
* Contributors: Vincent Rabaud

1.12.13 (2015-04-06)
--------------------

1.12.12 (2014-12-31)
--------------------
* Convert function to inline to avoid duplicates with image_transport
* Revert "remove GTK dependency"
  This reverts commit a6e15e796a40385fbbf8da05966aa47d179dcb46.
  Conflicts:
  image_view/CMakeLists.txt
  image_view/src/nodelets/disparity_nodelet.cpp
  image_view/src/nodes/stereo_view.cpp
* Revert "make sure waitKey is called after imshow"
  This reverts commit d13e3ed6af819459bca221ece779964a74beefac.
* Revert "brings back window_thread"
  This reverts commit 41a655e8e99910c13a3e7f1ebfdd083207cef76f.
* Contributors: Gary Servin, Vincent Rabaud

1.12.11 (2014-10-26)
--------------------
* brings back window_thread
  This fixes `#102 <https://github.com/ros-perception/image_pipeline/issues/102>`_ fully
* small optimizations
* add the image_transport parameter
* Contributors: Vincent Rabaud

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
* make sure waitKey is called after imshow
* remove GTK dependency
* small speedups
* Contributors: Vincent Rabaud

1.12.5 (2014-05-11)
-------------------
* image_view: Add depend on gtk2
* Contributors: Scott K Logan

1.12.4 (2014-04-28)
-------------------
* fixes `#65 <https://github.com/ros-perception/image_pipeline/issues/65>`_
* Contributors: Vincent Rabaud

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
* Added requirement for core.
* Contributors: Jonathan J Hunt

1.11.3 (2013-10-06 20:21:55 +0100)
----------------------------------
- #41: allow image_saver to save image topics
- #40: use proper download URL

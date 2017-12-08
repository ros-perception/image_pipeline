1.12.22 (2017-12-08)
--------------------

1.12.21 (2017-11-05)
--------------------
* [image_rotate] Added TF timeout so that transforms only need to be newer than last frame. (`#293 <https://github.com/ros-perception/image_pipeline/issues/293>`_)
* Contributors: mhosmar-cpr

1.12.20 (2017-04-30)
--------------------
* Fix CMake warnings about Eigen.
* address gcc6 build error
  With gcc6, compiling fails with `stdlib.h: No such file or directory`,
  as including '-isystem /usr/include' breaks with gcc6, cf.,
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70129.
  This commit addresses this issue for this package in the same way
  it was addressed in various other ROS packages. A list of related
  commits and pull requests is at:
  https://github.com/ros/rosdistro/issues/12783
  Signed-off-by: Lukas Bulwahn <lukas.bulwahn@oss.bmw-carit.de>
* Contributors: Lukas Bulwahn, Vincent Rabaud

1.12.19 (2016-07-24)
--------------------
* Fix frames if it is empty to rotate image
* Contributors: Kentaro Wada

1.12.18 (2016-07-12)
--------------------

1.12.17 (2016-07-11)
--------------------

1.12.16 (2016-03-19)
--------------------
* clean OpenCV dependency in package.xml
* Contributors: Vincent Rabaud

1.12.15 (2016-01-17)
--------------------

1.12.14 (2015-07-22)
--------------------

1.12.13 (2015-04-06)
--------------------

1.12.12 (2014-12-31)
--------------------

1.12.11 (2014-10-26)
--------------------

1.12.10 (2014-09-28)
--------------------

1.12.9 (2014-09-21)
-------------------

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
* use NODELET_** macros instead of ROS_** macros
* use getNodeHandle rather than getPrivateNodeHandle
* add executable to load image_rotate/image_rotate nodelet.
  add xml file to export nodelet definition.
  Conflicts:
  image_rotate/package.xml
* make image_rotate nodelet class
  Conflicts:
  image_rotate/CMakeLists.txt
  image_rotate/package.xml
  image_rotate/src/nodelet/image_rotate_nodelet.cpp
* move image_rotate.cpp to nodelet directory according to the directory convenstion of image_pipeline
* Contributors: Ryohei Ueda

1.12.1 (2014-04-06)
-------------------
* replace tf usage by tf2 usage

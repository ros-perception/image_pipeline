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

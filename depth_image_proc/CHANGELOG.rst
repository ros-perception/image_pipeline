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
* Add point_cloud_xyzi nodelet
  This is for cameras that output depth and intensity images.
  It's based on the point_cloud_xyzrgb nodelet.
* Missing runtime dependency - eigen_conversions
  `libdepth_image_proc` is missing this dependency at runtime
  ```
  > ldd libdepth_image_proc.so  | grep eigen
  libeigen_conversions.so => not found
  ```
  Which causes the following error on loading depth_image_proc:
  ```
  [ INFO] [1402564815.530736554]: /camera/rgb/camera_info -> /camera/rgb/camera_info
  [ERROR] [1402564815.727176562]: Failed to load nodelet [/camera/depth_metric_rect] of type
  [depth_image_proc/convert_metric]: Failed to load library /opt/ros/indigo/lib//libdepth_image_proc.so.
  Make sure that you are calling the PLUGINLIB_EXPORT_CLASS macro in the library code, and that
  names are consistent between this macro and your XML. Error string: Could not load library (Poco
  exception = libeigen_conversions.so: cannot open shared object file: No such file or directory)
  [FATAL] [1402564815.727410623]: Service call failed!
  ```
* Contributors: Daniel Stonier, Hunter Laux

1.12.4 (2014-04-28)
-------------------
* depth_image_proc: fix missing symbols in nodelets
* Contributors: Michael Ferguson

1.12.3 (2014-04-12)
-------------------

1.12.2 (2014-04-08)
-------------------

1.12.1 (2014-04-06)
-------------------
* replace tf usage by tf2 usage

1.12.0 (2014-04-04)
-------------------
* remove PCL dependency

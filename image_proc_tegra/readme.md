# image_proc_tegra

A ROS nodelet for image rectification using OpenCV's gpu functions. This nodelet is useful to reduce CPU usage in rectifying large video streams.
Needs OpenCV 3.4 with CUDA support


Install:

```
cd ~/catkin_ws/src
git clone https://github.com/DavidTorresOcana/image_pipeline
catkin_make
```

same topics as image_proc:

```
Subscribed topics:
  ~camera_info
  ~image_raw

Published topics:
  ~image_rect
```

## Nodelets 
### image_proc_tegra
Rectification with standard calibration method as in image_proc

example launch file:

```
  <!-- nodelet manager from image stream -->
  <node pkg="nodelet" type="nodelet" name="image_proc_tegra"  args="manager" />
  
  <node pkg="nodelet" type="nodelet" name="image_proc_test" args="load image_proc_tegra/RectifyNodelet camera_nodelet_manager" output="screen">
    <remap from="camera_info" to="/camera/color/camera_info" />
    <remap from="image_raw" to="/camera/color/image_raw" />
    <remap from="image_rect" to="/camera/color/image_rect" />
  </node>
```
### image_proc_tegra_fisheye
Rectification using OpenCV 3 fisheye camera rectification implementation. Works with any camera that was rectifies using camera_calibration_fisheye (https://github.com/DavidTorresOcana/image_pipeline/tree/indigo/camera_calibration_fisheye)

example launch file:
```
  <!-- nodelet manager from image stream -->
  <node pkg="nodelet" type="nodelet" name="image_proc_tegra_fisheye"  args="manager" />
  <node pkg="nodelet" type="nodelet" name="image_proc_test" args="load image_proc_tegra_fisheye/RectifyNodelet camera_nodelet_manager" output="screen">
    <remap from="camera_info" to="/camera/color/camera_info" />
    <remap from="image_raw" to="/camera/color/image_raw" />
    <remap from="image_rect" to="/camera/color/image_rect" />
  </node>
```

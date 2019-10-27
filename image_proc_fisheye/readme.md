#image_proc_fisheye

A ROS nodelet for image rectification of fisheye cameras images. Needs camera to be calibrated with OpenCV's 3.4+ fisheye model.

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


example launch file:
```
  <!-- nodelet manager from image stream -->
  <node pkg="nodelet" type="nodelet" name="camera_nodelet_manager"  args="manager" />
  <node pkg="nodelet" type="nodelet" name="image_proc_fisheye" args="load image_proc_fisheye/RectifyNodelet camera_nodelet_manager" output="screen">
    <remap from="camera_info" to="/camera/color/camera_info" />
    <remap from="image_raw" to="/camera/color/image_raw" />
    <remap from="image_rect" to="/camera/color/image_rect" />
  </node>
```

## Overview

This package contains the
[image_proc node](#image_proc),
which is meant to sit between the camera driver and vision processing
nodes. `image_proc` removes camera distortion from the raw image
stream, and if necessary will convert Bayer or YUV422 format image data
to color.

  -- ------------------------------------------------------------- -- ------------------------------------------------------------------------------------------------------------------------------ --
                                                                      <tablestyle="width: 99%">**Monocular processing example**                                                                  
     ![Raw image](doc/left_raw.png)                                   ![Rectified image](doc/left_rect_color.png)                                                                                    
     **image_raw**: Original camera image, Bayered and distorted      **image_rect_color**: Rectified image, de-Bayered and undistorted (amount of black border may vary depending on calibration)   
  -- ------------------------------------------------------------- -- ------------------------------------------------------------------------------------------------------------------------------ --



```wiki
<<Version(ROS Electric)>> `image_proc` also contains the [[image_proc#image_proc.2BAC8-electric.image_proc.2BAC8-crop_decimate|crop_decimate]] nodelet, which applies decimation and ROI to a raw image in software.
```



## Mini Tutorial

Make sure your camera driver is running. You can check
`rostopic list | grep image_raw` to see the available raw image topics
from compatible drivers.

Normally the raw image from the camera driver is not what you want for
visual processing, but rather an undistorted and (if necessary)
debayered image. This is the job of `image_proc`. If you are running
on a robot, it's probably best to run `image_proc` there. For
example, if the driver is publishing topics `/my_camera/image_raw` and
`/my_camera/camera_info` you would do:



```
$ ROS_NAMESPACE=my_camera rosrun image_proc image_proc
```



Notice that we push our `image_proc` instance down into the
`/my_camera` namespace, in which it subscribes to the `image_raw`
and `camera_info` topics. All output topics are likewise published
within the `/my_camera` namespace.

In a separate terminal (on your home machine, if you are running on a
robot): 

```
$ rosrun image_view image_view image:=my_camera/image_rect_color
```



This will display an undistorted color image from "my_camera".

## Nodes

### image_proc

![Monocular image node](doc/image_proc.png)

De-mosaics and undistorts the raw camera image stream.

All processing is on demand. Color processing is performed only if there
is a subscriber to a color topic. Rectification is performed only if
there is a subscriber to a rectified topic. While there are no
subscribers to output topics, `image_proc` unsubscribes from the
`image_raw` and `camera_info` topics.

[image_transport](https://index.ros.org/p/image_transport) is used for
all publications and subscriptions.

#### Subscribed Topics

- `image_raw`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  Raw image stream from the camera driver.
- `camera_info`
  ([sensor_msgs/msg/CameraInfo](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CameraInfo.html)):
  Camera metadata.

#### Published Topics

- `image_mono`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  Monochrome unrectified image.
- `image_rect`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  Monochrome rectified image.
- `image_color`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  Color unrectified image.
- `image_rect_color`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  Color rectified image.

#### Parameters

- `~queue_size` (`int`, default: `5`): Size of message queue for
  synchronizing image and camera_info topics. You may need to raise this
  if images take significantly longer to travel over the network than
  camera info.

## Nodelets

New in Diamondback, this package contains
[nodelets](https://index.ros.org/p/nodelet) for the tasks of debayering
and rectification. This allows `image_proc` functions to be easily
combined with other nodelets, for example camera drivers or higher-level
vision processing. In fact, the `image_proc` node simply loads one
`debayer` nodelet and two `rectify` nodelets.

### image_proc/debayer

Takes a raw camera stream and publishes monochrome and color versions of
it. If the raw images are Bayer pattern, it debayers using bilinear
interpolation.

#### Subscribed Topics

- `image_raw`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  Raw image stream from the camera driver.

#### Published Topics

- `image_mono`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  Monochrome image.
- `image_color`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  Color unrectified image.



```
{#!wiki version electric_and_newer

==== Parameters ====

===== Dynamically Reconfigurable Parameters =====

See the [[dynamic_reconfigure]] package for details on dynamically reconfigurable parameters.

 * {{{~debayer
```

 (`int`, default: `0`): Debayering algorithm.
Possible values are: \* `Bilinear` (0): Fast algorithm using bilinear
interpolation \* `EdgeAware` (1): Edge-aware algorithm \*
`EdgeAwareWeighted` (2): Weighted edge-aware algorithm \* `VNG` (3):
Slow but high quality Variable Number of Gradients algorithm

}}}}

### image_proc/rectify

Takes an unrectified image stream and its associated calibration
parameters, and produces rectified images.

#### Subscribed Topics

- `image_mono`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  Unrectified image stream.
- `camera_info`
  ([sensor_msgs/msg/CameraInfo](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CameraInfo.html)):
  Camera metadata.

#### Published Topics

- `image_rect`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  Rectified image.

#### Parameters

- `~queue_size` (`int`, default: `5`): Size of message queue for
  synchronizing image and camera_info topics. You may need to raise this
  if images take significantly longer to travel over the network than
  camera info.



```
{#!wiki version electric_and_newer

===== Dynamically Reconfigurable Parameters =====

See the [[dynamic_reconfigure]] package for details on dynamically reconfigurable parameters.

 * {{{~interpolation
```

 (`int`, default: `1`): Interpolation algorithm
between source image pixels. Possible values are: \* `NN` (0): Nearest
neighbor \* `Linear` (1): Linear \* `Cubic` (2): Cubic \*
`Lanczos4` (4): Lanczos4

}}}}



```
{#!wiki version electric_and_newer

=== image_proc/crop_decimate ===

Applies decimation (software binning) and ROI to a raw camera image post-capture. Remap `camera` and `camera_out` to the desired input/output camera namespaces.

==== Subscribed Topics ====

 * {{{camera/image_raw
```


([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
Raw image stream from the camera driver.

- `camera/camera_info`
  ([sensor_msgs/msg/CameraInfo](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CameraInfo.html)):
  Camera metadata.

#### Published Topics

- `camera_out/image_raw`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  Cropped and decimated "raw" image.
- `camera_out/camera_info`
  ([sensor_msgs/msg/CameraInfo](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CameraInfo.html)):
  Camera metadata, with binning and ROI fields adjusted to match output
  raw image.

#### Parameters

- `~queue_size` (`int`, default: `5`): Size of message queue for
  synchronizing image and camera_info topics. You may need to raise this
  if images take significantly longer to travel over the network than
  camera info.

##### Dynamically Reconfigurable Parameters

See the
[dynamic_reconfigure](https://index.ros.org/p/dynamic_reconfigure)
package for details on dynamically reconfigurable parameters.

- `~decimation_x` (`int`, default: `1`): Number of pixels to decimate to
  one horizontally. Range: 1 to 16
- `~decimation_y` (`int`, default: `1`): Number of pixels to decimate to
  one vertically. Range: 1 to 16
- `~x_offset` (`int`, default: `0`): X offset of the region of interest.
  Range: 0 to 2447
- `~y_offset` (`int`, default: `0`): Y offset of the region of interest.
  Range: 0 to 2049
- `~width` (`int`, default: `0`): Width of the region of interest.
  Range: 0 to 2448
- `~height` (`int`, default: `0`): Height of the region of interest.
  Range: 0 to 2050

}}}}



```
{#!wiki version fuerte_and_newer

 * {{{~interpolation
```

 (`int`, default: `0`): Sampling algorithm.
Possible values are: \* `NN` (0): Nearest-neighbor sampling \*
`Linear` (1): Bilinear interpolation \* `Cubic` (2): Bicubic
interpolation over 4x4 neighborhood \* `Area` (3): Resampling using
pixel area relation \* `Lanczos4` (4): Lanczos interpolation over 8x8
neighborhood

}}}}



```
{#!wiki version indigo_and_newer
<<Version(ROS Indigo)>>

=== image_proc/resize ===

Takes image and/or camera info and resize them.

==== Subscribed Topics ====

 * {{{image
```


([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
Arbitrary image.

- `camera_info`
  ([sensor_msgs/msg/CameraInfo](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CameraInfo.html)):
  Camera parameters.

#### Published Topics

- `~image`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  Resized image.
- `~camera_info`
  ([sensor_msgs/msg/CameraInfo](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CameraInfo.html)):
  Resized camera info.

}}}}

## Launch files

### image_proc.launch

Loads `debayer` and `rectify` nodelets for one camera into a
user-provided nodelet manager. The in-process equivalent of running the
`image_proc` node. Launch (or include in a higher-level launch file)
in the camera namespace, just as you would run `image_proc`.

#### Arguments

- `manager` (`string`): The name of the target nodelet manager. It will
  need to be globally qualified (e.g. `/my_manager`), unless the
  manager is already in the camera namespace.
- `respawn` (`bool`, default: `false`): If true, enables bond topics and
  respawns any nodelet loaders that die. If the manager also respawns,
  the system is robust to any node dying. However, the proliferation of
  bond topics can overwhelm introspection tools like [rxgraph](rxgraph).

#### Subscribed and Published Topics

Identical to the `image_proc` node.

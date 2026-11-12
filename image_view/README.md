## Usage

### Viewing a single image topic



```
rosrun image_view image_view image:=<image topic> [image transport type]
```



For example, to view raw images on the topic `/camera/image`, use:


```
rosrun image_view image_view image:=/camera/image
```



You may save the current image by right-clicking on the display window.
By default, images will be saved as `frame0000.jpg`,
`frame0001.jpg`, \....

If you want to view a compressed image stream (usually a good idea over
wireless!) using the capabilities of
[image_transport](https://index.ros.org/p/image_transport), specify the
transport type as a command-line argument. For example, if
[theora_image_transport](https://index.ros.org/p/theora_image_transport)
is built on the publisher's side, you can use `theora` transport:


```
rosrun image_view image_view image:=/camera/image theora
```



Note that this is merely shorthand equivalent to setting the
`image_transport` parameter: 

```
rosrun image_view image_view image:=/camera/image _image_transport:=theora
```



### Viewing stereo images



```
rosrun image_view stereo_view stereo:=<stereo namespace> image:=<image topic identifier>
```



For example, to view stereo image pairs on topics
`/my_stereo_cam/left/image_rect_color` and
`/my_stereo_cam/right/image_rect_color`, use: 

```
rosrun image_view stereo_view stereo:=/my_stereo_cam image:=image_rect_color
```



`stereo_view` also shows the disparity image computed from the stereo
pair, color-mapped for clarity.

You may save the current image pair by right-clicking on any display
window. By default, images will be saved as `left0000.jpg`,
`right0000.jpg`, `disp0000.jpg`, `left0001.jpg`,
`right0001.jpg`, `disp0001.jpg`\.... As with `image_view`, you can
specify an image transport to use for the left and right image as an
optional argument.

## Nodes

### image_view

Simple image viewer for ROS
[sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)
topics.

#### Subscribed Topics

- `image`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  The image topic. Should be remapped to the name of the real image
  topic.

#### Parameters

- `~autosize` (`bool`, default: `false`): Whether the window should
  autosize itself to the image or be resizeable by the user.
- `~filename_format` (`string`, default: `"frame%04i.jpg"`):
  printf-style format for saved image names. Use to control name,
  location and format of saved images.
- `~image_transport` (`string`, default: `"raw"`): Transport used
  for the image stream. `image_view` allows you to specify this as a
  simple command-line argument for convenience.
- `~window_name` (`string`, default: `name of the image topic`): The
  name of the display window.

### disparity_view

Simple viewer for
[stereo_msgs/msg/DisparityImage](https://docs.ros.org/en/rolling/p/stereo_msgs/msg/DisparityImage.html)
topics. Color-maps the disparity image for visualization.

#### Subscribed Topics

- `image`
  ([stereo_msgs/msg/DisparityImage](https://docs.ros.org/en/rolling/p/stereo_msgs/msg/DisparityImage.html)):
  The disparity image topic. Should be remapped to the name of the real
  topic.

#### Parameters

- `~autosize` (`bool`, default: `false`): Whether the window should
  autosize itself to the image or be resizeable by the user.
- `~window_name` (`string`, default: `name of the image topic`): The
  name of the display window.

### stereo_view

Viewer for stereo images. Shows the synchronized left/right image pair
and the disparity image (color-mapped) computed from them.

#### Subscribed Topics

- `<stereo>/left/<image>`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  The left image topic. Formal parameters `stereo` and `image`
  should be remapped appropriately.
- `<stereo>/right/<image>`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  The right image topic. Formal parameters `stereo` and `image`
  should be remapped appropriately.
- `<stereo>/disparity`
  ([stereo_msgs/msg/DisparityImage](https://docs.ros.org/en/rolling/p/stereo_msgs/msg/DisparityImage.html)):
  The disparity image computed from the left/right stereo pair.

#### Parameters

- `~autosize` (`bool`, default: `true`): Whether the windows should
  autosize to the image or be resizeable by the user.
- `~filename_format` (`string`, default: `"%s%04i.jpg"`):
  printf-style format for saved image names. Use to control name,
  location and format of saved images. The string argument is
  `"left"` or `"right"`.
- `~image_transport` (`string`, default: `"raw"`): Transport used
  for the image streams.
- `~approximate_sync` (`bool`, default: `false`): Whether to use
  approximate synchronization. Set to true if the left and right cameras
  do not produce exactly synced timestamps.
- `~queue_size` (`int`, default: `5`): Size of message queue for each
  synchronized topic. You may need to raise this if disparity processing
  takes too long, or if there are significant network delays.

## Nodelets

### image_view/image

Nodelet version of image_view. Brings up a display window for a
[sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)
topic.

#### Subscribed Topics

- `image`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  The image topic. Should be remapped to the name of the real image
  topic.

#### Parameters

- `~autosize` (`bool`, default: `false`): Whether the window should
  autosize itself to the image or be resizeable by the user.
- `~filename_format` (`string`, default: `"frame%04i.jpg"`):
  printf-style format for saved image names. Use to control name,
  location and format of saved images.
- `~image_transport` (`string`, default: `"raw"`): Transport used
  for the image stream. `image_view` allows you to specify this as a
  simple command-line argument for convenience.
- `~window_name` (`string`, default: `name of the image topic`): The
  name of the display window.

### image_view/disparity

Nodelet version of disparity_view. Brings up a display window for a
[stereo_msgs/msg/DisparityImage](https://docs.ros.org/en/rolling/p/stereo_msgs/msg/DisparityImage.html)
topic, color-mapped for visualization.

#### Subscribed Topics

- `image`
  ([stereo_msgs/msg/DisparityImage](https://docs.ros.org/en/rolling/p/stereo_msgs/msg/DisparityImage.html)):
  The disparity image topic. Should be remapped to the name of the real
  topic.

#### Parameters

- `~autosize` (`bool`, default: `false`): Whether the window should
  autosize itself to the image or be resizeable by the user.
- `~window_name` (`string`, default: `name of the image topic`): The
  name of the display window.



```
{{{#!wiki version hydro_and_newer
== Tools ==


=== image_saver ===

This tool allows you to save images as jpg/png file from streaming (ROS <<MsgLink(sensor_msgs/Image)>> topic) to a file. From command line, you can run by `rosrun image_view image_saver image:=[your topic]`, or see [[http://answers.ros.org/question/9377/how-to-take-save-a-snapshot-from-my-kinect-to-disk/?answer=235294#post-id-235294|this answer]] to control the timing of capture.

==== Subscribed Topics ====

 * {{{image
```


([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
The image topic. Should be remapped to the name of the real image topic.

#### Services

- `save`
  ([std_srvs/srv/Empty](https://docs.ros.org/en/rolling/p/std_srvs/srv/Empty.html)):
  Save images, you need to set save_all_images to false

#### Parameters

- `~filename_format` (`string`, default: `left%04d.%s`): File name for
  saved images, you can use '%04i' for sequence number, and '%s' for
  default file format, you can use 'jpg' ,'png', 'pgm' as filename
  suffixes.
- `~encoding` (`string`, default: `'bgr8'`): Encoding type of input
  image topic.
- `~save_all_image` (`bool`, default: `true`): If you set false, images
  are only saved when 'save' service is called

### extract_images

This tool also allows you to save images as jpg/png file from streaming
(ROS
[sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)
topic) to a file. `image_saver` node provide very similar
functionalities, such as providing service call to trigger the node to
save images, save images other than Jpeg format, etc.

#### Subscribed Topics

- `image`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  The image topic. Should be remapped to the name of the real image
  topic.

#### Parameters

- `~filename_format` (`string`, default: `frame%04d.jpg`): File name
  for saved images, you must add use '%04i' for sequence number.
- `~sec_per_frame` (`double`, default: `'0.1'`): set sec per frame
  value.

### video_recorder

This tool allows you to record a video stream (ROS
[sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)
topic) to a file. It relies on
[OpenCV's VideoWriter class](http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html#videowriter).
With the default options, it encodes the video as MPG, encapsulated in a
AVI container at 15 fps, and produces a file called `output.avi` in
the current directory.

#### Subscribed Topics

- `image`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  The image topic. Should be remapped to the name of the real image
  topic.

#### Parameters

- `~filename` (`string`, default: `output.avi`): Path and name of the
  output video.
- `~fps` (`int`, default: `15`): Framerate of the video.
- `~codec` (`string`, default: `MJPG`): The
  [FOURCC](http://www.fourcc.org/codecs.php) identifier of the codec.
- `~encoding` (`string`, default: `bgr8`): The image color space of
  the video.

}}}}}}

## Troubleshooting

### Errors when using package

#### Error: Received error message in header\... Dropping connection

**Example:** 

```
[ERROR] 1260927377.348922000: Received error message in header for connection to [TCPROS connection to [10.0.0.136:39531 on socket 17]]: [Client [/image_view_1260927377013984000] wants topic /image/compressed to have datatype/md5sum [sensor_msgs/Image/060021388200f6f0f447d0fcd9c64743], but our version has [sensor_msgs/CompressedImage/8f7a12909da2c9d3332d540a0977563f]. Dropping connection.]
```



Above that error you might see a warning: 

```
[ WARN] 1260927377.319546000: [image_transport] It looks like you are trying to subscribe directly to a transport-specific image topic '/image/compressed', in which case you will likely get a connection error. Try subscribing to the base topic '/image' instead with parameter ~image_transport set to 'compressed' (on the command line, _image_transport:=compressed). See http://ros.org/wiki/image_transport for details.
```



**Solution:**

- Are you subscribing to an image topic? If the data type of the topic
  you remapping `image` to is not
  [sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html),
  ROS will detect the mismatch and drop the connection. For example:
  

```
$ rostopic type image/compressed
sensor_msgs/CompressedImage
# Does not match sensor_msgs/Image
```


- Are you subscribing to a transport-specific subtopic? For example,
  `image_view image:=camera/image/theora`? This is a corollary of the
  previous case; the data type of a transport-specific topic like
  `camera/image/theora` is not
  [sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)
  but rather some packet message type. If you want to view the base
  topic `camera/image` using theora streaming video, give the
  transport name as a separate argument: 

```
$ rosrun image_view image_view image:=camera/image theora
```

 That
  tells `image_view` to load the theora plugin to
  [image_transport](https://index.ros.org/p/image_transport) and
  subscribe to the appropriate subtopic.

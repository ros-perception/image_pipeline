Command line tool to republish an image topic flipped (horizontally, vertically or both).

Useful when you accidentally mounted your camera upside down or you are given a mirrored webcam image, for example.

Usage:

	rosrun image_flip image_flip image:=<image topic> flipped:=<flipped image topic> <h(orizontal)/v(ertical)/b(oth)>

For example:

	rosrun image_flip image_flip image:=/camera/image_raw flipped:=/camera/image_vertical_flip_raw v

You can view it with:

	rosrun image_view image_view image:=/camera/image_vertical_flip_raw

Or:

	rosrun rqt_image_view rqt_image_view

And choose from the dropdown list the topic.

It's intended to use when your needs are simpler than what the amazing [image_rotate](http://wiki.ros.org/image_rotate) node does.

# This message defines all the information necessary to reconstruct a
# full set of stereo information.  It should be generated directly by
# a driver connected to a set of stereo cameras.  It is only intended
# to be fed into a stereo processing node, and should not otherwise be
# used.  This is the preferred message to log when generating log
# files, as it is the minimal representation of the information.

uint8       NONE=0
uint8       IMAGE_RAW=1
uint8       IMAGE=2
uint8       IMAGE_COLOR=3
uint8       IMAGE_RECT=4
uint8       IMAGE_RECT_COLOR=5

Header        header
StereoInfo    stereo_info
sensor_msgs/CameraInfo    left_info
uint8         left_type
sensor_msgs/Image         left_image
sensor_msgs/CameraInfo    right_info
uint8         right_type
sensor_msgs/Image         right_image
uint8         has_disparity
DisparityInfo disparity_info
sensor_msgs/Image         disparity_image

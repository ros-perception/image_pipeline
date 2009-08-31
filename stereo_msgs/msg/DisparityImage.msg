# This message contains an uncompressed disparity (16-bit) image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of cameara
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# specifics for disparity
# dpp is the interpolation factor, e.g., dpp=16 means each disparity increment
#   is 1/16 of a pixel
uint32 dpp

# maximum search range for disparities, in pixels
uint32 num_disp

# subwindow of valid stereo values
uint32 im_Dtop
uint32 im_Dleft
uint32 im_Dwidth
uint32 im_Dheight

# The legal values for encoding are in file src/image_encodings.cpp
# Should be 16bit monochrome
string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in src/image_encodings.cpp
uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

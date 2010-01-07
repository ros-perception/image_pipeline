# Separate header for compatibility with current TimeSynchronizer.
# Likely to be removed in a later release, use image.header instead.
Header header

# Floating point disparity image. The disparities are pre-adjusted for any
# x-offset between the principal points of the two cameras (in the case
# that they are verged). That is: d = x_l - x_r - (cx_l - cx_r)
sensor_msgs/Image image

# Stereo geometry. For disparity d, the depth from the camera is Z = fT/d.
float64 f # Focal length, pixels
float64 T # Baseline, world units

# Subwindow of valid stereo values.
sensor_msgs/RegionOfInterest valid_window

# The horopter, or 3D volume covered by the disparity search. Points with Z
# less than min_depth or greater than max_depth could not be found.
float64 min_depth
float64 max_depth

# Smallest allowed disparity increment. The smallest achievable depth range
# resolution is delta_Z = (Z^2/fT)*delta_d.
float64 delta_d

import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
assert cv2.__version__[0] == '3', 'The fisheye module requires opencv version >= 3.0.0'
import numpy as np
import os
from os import listdir
import glob

# You should replace these 3 lines with the output in calibration step
DIM=(1920, 1208)
K=np.array([[695.938094325298, 0.0, 947.823102161717], [0.0, 696.8169447082668, 651.7314027994207], [0.0, 0.0, 1.0]])
D=np.array([[-0.02782443761588723], [-0.03224246650257987], [0.016825224749675603], [-0.0035259044552496525]])

def undistort(img_path):
    img = cv2.imread(img_path)
    h,w = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    cv2.imshow("undistorted", undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
if __name__ == '__main__':
    for p in sys.argv[1:]:
        undistort(p)

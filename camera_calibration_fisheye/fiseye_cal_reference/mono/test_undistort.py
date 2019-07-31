import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
assert cv2.__version__[0] == '3', 'The fisheye module requires opencv version >= 3.0.0'
import numpy as np
import os
from os import listdir
import glob

# You should replace these 3 lines with the output in calibration step
DIM=(800, 503)
K=np.array([[290.0609844653627, 0.0, 399.7119405014219], [0.0, 289.1594924628207, 274.1984860354366], [0.0, 0.0, 1.0]])
D=np.array([[-0.03122721497580657], [-0.020698242172871275], [0.0035143644102214144], [0.0006258984511625903]])

# DIM=(1920, 1208)
# K=np.array([[696.0621148872598, 0.0, 947.6971496853826], [0.0, 696.9356605571118, 651.8101521495024], [0.0, 0.0, 1.0]])
# D=np.array([[-0.02814215088649154], [-0.03164955633645717], [0.016420714751802366], [-0.003441866971663222]])

methods = [("area", cv2.INTER_AREA), 
         ("nearest", cv2.INTER_NEAREST), 
         ("linear", cv2.INTER_LINEAR), 
         ("cubic", cv2.INTER_CUBIC), 
         ("lanczos4", cv2.INTER_LANCZOS4)]
    
def undistort(img_path):
    img = cv2.imread(img_path)
    h,w = img.shape[:2]
    
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = []
    for i,m in enumerate(methods):
        print  "interpolation ", m[0]
        undistorted_img = cv2.remap(img, map1, map2, interpolation=m[1], borderMode=cv2.BORDER_CONSTANT)
        img = np.hstack((img, undistorted_img))
        print img.shape
    
    cv2.imshow("undistorted", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
if __name__ == '__main__':
    for p in sys.argv[1:]:
        undistort(p)

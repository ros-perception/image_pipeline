import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
assert cv2.__version__[0] == '3', 'The fisheye module requires opencv version >= 3.0.0'
import numpy as np
import os
from os import listdir
import glob

# You should replace these 3 lines with the output in calibration step
# DIM=(800, 503)
# K=np.array([[290.0609844653627, 0.0, 399.7119405014219], [0.0, 289.1594924628207, 274.1984860354366], [0.0, 0.0, 1.0]])
# D=np.array([[-0.03122721497580657], [-0.020698242172871275], [0.0035143644102214144], [0.0006258984511625903]])

DIM=(1920, 1208)
K=np.array([[696.0621148872598, 0.0, 947.6971496853826], [0.0, 696.9356605571118, 651.8101521495024], [0.0, 0.0, 1.0]])
D=np.array([[-0.02814215088649154], [-0.03164955633645717], [0.016420714751802366], [-0.003441866971663222]])

def undistort(img_path, balance=0.0, dim2=None, dim3=None):
    img = cv2.imread(img_path)
    dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
    assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if not dim2:
        dim2 = dim1

    if not dim3:
        dim3 = dim1
        
    scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
    print "dim1 ",dim1, "dim2 ",dim2,  "dim3 ",dim3 
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=1.0)
    new_K[0][2] = int(dim2[0]/2) - 0# Set manually the center of the img in New Camera matrix. X axis
    new_K[1][2] = int(dim2[1]/2) + 0 # Y-axis
    
    ######
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), scaled_K, dim3, cv2.CV_16SC2)
    #print new_K
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT)
    
    ##
    ##undistorted_img = cv2.resize(undistorted_img,(800, 503),interpolation=cv2.INTER_CUBIC)
    
    cv2.imshow("undistorted", undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
if __name__ == '__main__':
    if len(sys.argv)==2:
        undistort( sys.argv[1])
    elif len(sys.argv)==3:
        undistort( sys.argv[1],dim2=tuple(  map(int, sys.argv[2].split(',') ) ) )
    elif len(sys.argv)==4:
        undistort( sys.argv[1],dim2=tuple(  map(int, sys.argv[2].split(',') ) ),dim3=tuple(  map(int, sys.argv[3].split(',') ) ) )
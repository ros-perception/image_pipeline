import sys
import cv2
import numpy as np
import os
from os import listdir

def lryaml( name, d, k, r, p): # camera_name, distortion, intrinsics, R, P ...   name, d, k, r, p
    calmessage = (""
    + "image_width: " + str( size[0]) + "\n"
    + "image_height: " + str( size[1]) + "\n"
    + "camera_name: " + name + "\n"
    + "camera_matrix:\n"
    + "  rows: 3\n"
    + "  cols: 3\n"
    + "  data: [" + ", ".join(["%8f" % i for i in k.reshape(1,9)[0]]) + "]\n"
    + "distortion_model: " + ("rational_polynomial" if d.size > 5 else "plumb_bob") + "\n"
    + "distortion_coefficients:\n"
    + "  rows: 1\n"
    + "  cols: 5\n"
    + "  data: [" + ", ".join(["%8f" % d[i,0] for i in range(d.shape[0])]) + "]\n"
    + "rectification_matrix:\n"
    + "  rows: 3\n"
    + "  cols: 3\n"
    + "  data: [" + ", ".join(["%8f" % i for i in r.reshape(1,9)[0]]) + "]\n"
    + "projection_matrix:\n"
    + "  rows: 3\n"
    + "  cols: 4\n"
    + "  data: [" + ", ".join(["%8f" % i for i in p.reshape(1,12)[0]]) + "]\n"
    + "")
    return calmessage
        

CHECKERBOARD = (6,8)
subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW
objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

origin_dir = 'calibrationdata/'

images = os.listdir(origin_dir)

_img_shape = None
for fname in images:
    if fname.endswith(".png"):
        img = cv2.imread(origin_dir+fname)
        if _img_shape == None:
            _img_shape = img.shape[:2]
        else:
            assert _img_shape == img.shape[:2], "All images must share the same size."
        
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
        
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
            imgpoints.append(corners)

N_OK = len(objpoints)
K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]

rms, _, _, _, _ = cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        gray.shape[::-1],
        K,
        D,
        rvecs,
        tvecs,
        calibration_flags,
        (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )
    
# R is identity matrix for monocular calibration
R = np.eye(3, dtype=np.float64)
P = np.zeros((3, 4), dtype=np.float64)

print("Found " + str(N_OK) + " valid images for calibration")
print("DIM=" + str(_img_shape[::-1]))
print("K=np.array(" + str(K.tolist()) + ")")
print("D=np.array(" + str(D.tolist()) + ")")
print("R=np.array(" + str(R) + ")")
print("T=np.array(" + str(P) + ")")


# Write to Yaml

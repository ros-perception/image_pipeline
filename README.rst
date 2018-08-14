Fork of image_pipeline


.. image:: https://travis-ci.org/ros-perception/image_pipeline.svg?branch=indigo
    :target: https://travis-ci.org/ros-perception/image_pipeline

This package fills the gap between getting raw images from a camera driver and higher-level vision processing.

This Fork adds:

* camera_calibration_fisheye(https://github.com/DavidTorresOcana/image_pipeline/tree/indigo/camera_calibration_fisheye): A calibration tool for fisheye cameras. It uses OpenCV >3.

* image_proc_tegra(https://github.com/DavidTorresOcana/image_pipeline/tree/indigo/image_proc_tegra):
- A camera rectification package for camera rectification with OpenCV >3 and CUDA support. Tested in Tegra cores. Faster than image_proc since it uses CUDA for rectification
It also implements a Nodelet for Fisheye cameras rectification, if calibrated with camera_calibration_fisheye(https://github.com/DavidTorresOcana/image_pipeline/tree/indigo/camera_calibration_fisheye). It needs OpenCV 3 with CUDA suport
 
 
 




* image_proc_fisheye (https://github.com/DavidTorresOcana/image_pipeline/tree/indigo/image_proc_fisheye): Same as image_proc(https://github.com/DavidTorresOcana/image_pipeline/tree/indigo/image_proc) but for fisheye cameras calibrated with camera_calibration_fisheye(https://github.com/DavidTorresOcana/image_pipeline/tree/indigo/camera_calibration_fisheye). It does not need CUDA suport



Fork of image_pipeline
==============

.. image:: https://travis-ci.org/ros-perception/image_pipeline.svg?branch=indigo
    :target: https://travis-ci.org/ros-perception/image_pipeline

This package fills the gap between getting raw images from a camera driver and higher-level vision processing.

This Fork adds:

* image_proc_tegra: A camera rectification package for camera rectification with OpenCV >3 and CUDA support. Tested in Tegra cores. Faster than image_proc since it uses CUDA for rectification
 - It also implements a Nodelet for Fish eye cameras rectification, if calibrated with camera_calibration_fisheye
 
* camera_calibration_fisheye: A calibration tool for fisheye cameras. It uses OpenCV >3.

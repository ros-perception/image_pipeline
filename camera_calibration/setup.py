#!/usr/bin/env python
from setuptools import setup, find_packages

PACKAGE_NAME = "camera_calibration"

setup(
    name=PACKAGE_NAME,
    version='1.12.23',
    packages=["camera_calibration", "camera_calibration.nodes"],
    py_modules=[],
    package_dir={'': 'src'},
    install_requires=[
        'setuptools',
        'opencv-python'
    ],
    zip_safe=True,
    author='James Bowman, Patrick Mihelich',
    maintainer='Vincent Rabaud',
    maintainer_email='vincent.rabaud@gmail.com',
    keywords=['ROS2'],
    description='Camera_calibration allows easy calibration of monocular or stereo cameras using a checkerboard calibration target.',
    license='BSD',
    tests_require=[
        'pytest',
        'requests'
    ],
    entry_points={
        'console_scripts': [
            'cameracalibrator = camera_calibration.nodes.cameracalibrator:main',
            'cameracheck = camera_calibration.nodes.cameracheck:main',
        ],
    },
)

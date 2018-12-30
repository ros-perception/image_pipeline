#!/usr/bin/env python
from setuptools import setup, find_packages

setup(
    name='camera_calibration',
    version='1.12.23',
    packages=find_packages(),
    py_modules=[],
    install_requires=[
        'setuptools',
        'opencv-python'
    ],
    zip_safe=True,
    author='James Bowman, Patrick Mihelich',
    maintainer='Vincent Rabaud',
    maintainer_email='vincent.rabaud@gmail.com',
    keywords=['ROS'],
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

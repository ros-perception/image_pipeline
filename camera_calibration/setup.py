#!/usr/bin/env python
from setuptools import setup, find_packages

PACKAGE_NAME = "camera_calibration"

setup(
    name=PACKAGE_NAME,
    version='2.3.0',
    packages=["camera_calibration", "camera_calibration.nodes"],
    data_files=[
    ('share/ament_index/resource_index/packages',
      ['resource/' + PACKAGE_NAME]),
    ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    py_modules=[],
    package_dir={'': 'src'},
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    author='James Bowman, Patrick Mihelich',
    maintainer='Vincent Rabaud, Steven Macenski, Joshua Whitley',
    maintainer_email='vincent.rabaud@gmail.com, stevenmacenski@gmail.com, whitleysoftwareservices@gmail.com',
    keywords=['ROS2'],
    description='Camera_calibration allows easy calibration of monocular or stereo cameras using a checkerboard calibration target .',
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

#!/usr/bin/env python
from setuptools import setup

PACKAGE_NAME = 'camera_calibration'

setup(
    name=PACKAGE_NAME,
    version='8.0.1',
    packages=['camera_calibration', 'camera_calibration.nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    py_modules=[],
    package_dir={'': 'src'},
    install_requires=[
        'setuptools',
    ],
    extras_require={
        'test': ['pytest', 'requests'],
    },
    zip_safe=True,
    author='James Bowman, Patrick Mihelich',
    maintainer='Vincent Rabaud, Steven Macenski',
    maintainer_email='vincent.rabaud@gmail.com, stevenmacenski@gmail.com',
    keywords=['ROS2'],
    description='Camera_calibration allows easy calibration of monocular or stereo cameras '
                'using a checkerboard calibration target.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'cameracalibrator = camera_calibration.nodes.cameracalibrator:main',
            'cameracheck = camera_calibration.nodes.cameracheck:main',
            'tarfile_calibration = camera_calibration.nodes.tarfile_calibration:main',
        ],
    },
)

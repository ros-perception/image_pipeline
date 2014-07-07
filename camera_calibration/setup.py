#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['camera_calibration']
d['package_dir'] = {'':'src'}
d['scripts'] = ['nodes/cameracalibrator.py',
                'nodes/cameracheck.py',
                'nodes/camera_hammer.py',
                'scripts/tarfile_calibration.py']

setup(**d)

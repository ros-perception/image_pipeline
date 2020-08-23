#!/usr/bin/env python
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['camera_calibration']
d['package_dir'] = {'':'src'}

setup(**d)

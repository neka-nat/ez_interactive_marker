#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    scripts=['nodes/ez_interactive_marker'],
    packages=['ez_interactive_marker'],
    package_dir={'': 'src'})

setup(**d)

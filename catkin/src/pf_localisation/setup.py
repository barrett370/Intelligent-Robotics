#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup # pyre-ignore

d = generate_distutils_setup(
   packages=['pf_localisation'],
   package_dir={'': 'src'}
)

setup(**d)
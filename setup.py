import sys
from skbuild import setup
from setuptools import find_packages

# TODO enable cross-platform support for C/C++ extensions.
skip_cmake = False if sys.platform in ['linux'] else True

setup(
    packages=find_packages(where="python"),
    package_dir={"": "python"},
    include_package_data=True,
    package_data={"upsp.processing": ["templates/*.template"]},
    skip_cmake=skip_cmake
)

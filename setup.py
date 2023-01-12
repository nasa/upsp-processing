from skbuild import setup
from setuptools import find_packages

setup(
    packages=find_packages(where="python"),
    package_dir={"": "python"},
    include_package_data=True,
    package_data={"upsp.processing": ["templates/*.template"]},
)

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['joint_state_publisher'],
    package_dir={'': 'src'}
)

setup(**d)

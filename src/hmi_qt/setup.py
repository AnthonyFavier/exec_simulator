from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['hmi_qt', 'hmi_qt.widget',
              'hmi_qt.publisher'],
    package_dir={'': 'src'},
)

setup(**d)

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['plane_inspection'],
    package_dir={'': 'src'}
)
setup(**d)

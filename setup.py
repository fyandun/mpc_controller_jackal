
from catkin_pkg.python_setup import generate_distutils_setup
from distutils.core import setup


requirements = [
    'numpy',
    'scikit-learn',
]

setup_args = generate_distutils_setup(name='mpc_skid_steer',
      version='1.0.0',
      description='MPC controller for a skid steer robot',
      author='',
      author_email='',
      package_dir = {'': '.'},
      packages=['robot_common'],
      install_requires = requirements,
     )

setup(**setup_args)




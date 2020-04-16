#
# install with:
#   sudo python3 setup.py build install --force
#
# Anaconda (Windows):
# 1.  conda install m2w64-toolchain
#     conda install -c anaconda libpython
#     conda install -c msys2 m2w64-toolchain
# 2.  echo [build] > %CONDA_PREFIX%\Lib\distutils\distutils.cfg
#     echo compiler = mingw32 >> %CONDA_PREFIX%\Lib\distutils\distutils.cfg
# 3. install (see above) will generate d:\Users\...\Miniconda3\envs\py36\Lib\site-packages\acado.cp36-win_amd64.pyd

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




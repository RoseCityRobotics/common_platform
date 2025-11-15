import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pos_to_vel'

setup(
  name=package_name,
  version='0.0.1',
  packages=find_packages(exclude=['test']),
  data_files=[
    ('share/ament_index/resource_index/packages',
      ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='Joe Cole',
  maintainer_email='joe@rosecityrobotics.com',
  description='Odometry calibration node with discrete keyboard teleop',
  license='MIT',
  tests_require=['pytest'],
  entry_points={
    'console_scripts': [
      'odom_calibration_node = pos_to_vel.odom_calibration_node:main',
    ],
  },
)


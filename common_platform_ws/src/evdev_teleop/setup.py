from setuptools import find_packages, setup

package_name = 'evdev_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/evdev_teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joe Cole',
    maintainer_email='joe@rosecityrobotics.com',
    description='Allow robot teleoperation using a Logitech keyboard connected to a USB wireless keyboard dongle.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'evdev_teleop = evdev_teleop.ros2_evdev_teleop:main',
        ],
    },
)

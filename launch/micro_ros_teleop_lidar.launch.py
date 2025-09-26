#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
  # micro-ROS agent via Docker (uses provided env.list relative to current working dir)
  micro_ros_agent = ExecuteProcess(
    cmd=[
      'sudo', 'docker', 'run', '-it', '--rm',
      '-v', '/dev:/dev', '--privileged', '--net=host',
      '--env-file', './env.list',
      'microros/micro-ros-agent:kilted',
      'serial', '--dev', '/dev/serial/by-id/usb-Teensyduino_Dual_Serial_14512140-if00',
      '-v4'
    ],
    output='screen',
    emulate_tty=True
  )

  # Teleop node (evdev_teleop)
  teleop_node = Node(
    package='evdev_teleop',
    executable='evdev_teleop',
    output='screen'
  )

  # RPLidar A1 include
  rplidar_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py'
      )
    ]),
    launch_arguments={
      'serial_port': '/dev/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
      'serial_baudrate': '115200',
      'frame_id': 'laser_frame'
    }.items()
  )

  actions = [
    micro_ros_agent,
    teleop_node,
    rplidar_launch
  ]

  ns = os.environ.get('ROS_NAMESPACE', '').strip()
  if ns:
    return LaunchDescription([
      GroupAction([
        PushRosNamespace(ns),
        *actions
      ])
    ])
  else:
    return LaunchDescription(actions)



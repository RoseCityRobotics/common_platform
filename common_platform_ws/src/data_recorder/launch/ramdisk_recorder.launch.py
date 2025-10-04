#!/usr/bin/env python3
"""
Simple RAM disk data recording launch file.

This launch file handles RAM disk mounting and data recorder startup
without requiring separate Python scripts.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
  # Declare launch arguments
  ramdisk_size_arg = DeclareLaunchArgument(
    'ramdisk_size',
    default_value='2G',
    description='Size of the RAM disk (e.g., 2G, 4G)'
  )
  
  ramdisk_path_arg = DeclareLaunchArgument(
    'ramdisk_path',
    default_value='/mnt/recording_ramdisk',
    description='Path where RAM disk will be mounted'
  )
  
  auto_start_arg = DeclareLaunchArgument(
    'auto_start',
    default_value='true',
    description='Automatically start recording when launched'
  )
  
  # Get launch configurations
  ramdisk_size = LaunchConfiguration('ramdisk_size')
  ramdisk_path = LaunchConfiguration('ramdisk_path')
  auto_start = LaunchConfiguration('auto_start')
  
  # Handle namespace for topics
  ns = os.environ.get('ROS_NAMESPACE', '').strip()
  if ns:
    camera_topic = f'{ns}/camera/image_raw'
    odom_topic = f'{ns}/odom'
  else:
    camera_topic = '/camera/image_raw'
    odom_topic = '/odom'

  print(f"Camera topic with namespace: {camera_topic}")
  print(f"Odometry topic with namespace: {odom_topic}")
  
  # Note: RAM disk mounting is handled by the data_recorder_node C++ code
  
  # Data recorder node
  data_recorder_node = Node(
    package='data_recorder',
    executable='data_recorder_node',
    name='data_recorder',
    output='screen',
    parameters=[{
      'output_dir': '/home/rcr/teleop_data',
      'camera_topic': camera_topic,
      'odom_topic': odom_topic,
      'record_rate': 30.0,
      'auto_start': auto_start,
    }],
  )
  
  # Create launch description
  launch_description = LaunchDescription([
    ramdisk_size_arg,
    ramdisk_path_arg,
    auto_start_arg,
  ])
  
  # Add namespace handling if needed
  if ns:
    launch_description.add_action(
      GroupAction([
        PushRosNamespace(ns),
        data_recorder_node
      ])
    )
  else:
    launch_description.add_action(data_recorder_node)
  
  return launch_description

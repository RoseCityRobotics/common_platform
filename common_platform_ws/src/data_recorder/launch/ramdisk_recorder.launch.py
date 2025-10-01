#!/usr/bin/env python3
"""
Launch file for RAM disk data recording.

This launch file starts the data recorder with automatic RAM disk mounting
and provides easy control via the ramdisk_control.py script.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
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
  
  backup_path_arg = DeclareLaunchArgument(
    'backup_path',
    default_value='/home/rcr/robot_data',
    description='Path where data will be backed up on SD card'
  )
  
  namespace_arg = DeclareLaunchArgument(
    'namespace',
    default_value='',
    description='ROS namespace for multi-robot setups'
  )
  
  auto_start_arg = DeclareLaunchArgument(
    'auto_start',
    default_value='false',
    description='Automatically start recording when launched'
  )
  
  # Get launch configurations
  ramdisk_size = LaunchConfiguration('ramdisk_size')
  ramdisk_path = LaunchConfiguration('ramdisk_path')
  backup_path = LaunchConfiguration('backup_path')
  namespace = LaunchConfiguration('namespace')
  auto_start = LaunchConfiguration('auto_start')
  
  # Handle namespace for topics
  ns = os.environ.get('ROS_NAMESPACE', '').strip()
  if ns:
    camera_topic = f'/{ns}/camera/image_raw'
    cmd_vel_topic = f'/{ns}/cmd_vel'
  else:
    camera_topic = '/camera/image_raw'
    cmd_vel_topic = '/cmd_vel'
  
  # Data recorder node (will be started by ramdisk_recorder.py)
  # This is just for reference - the actual node is started by the Python script
  data_recorder_node = Node(
    package='data_recorder',
    executable='data_recorder_node',
    name='data_recorder',
    output='screen',
    parameters=[{
      'output_dir': ramdisk_path,
      'camera_topic': camera_topic,
      'cmd_vel_topic': cmd_vel_topic,
      'record_rate': 30.0,
      'auto_start': auto_start,
      'camera_type': 'libcamera',
    }],
    condition=IfCondition('false')  # Disabled - started by Python script instead
  )
  
  # Create launch description
  launch_description = LaunchDescription([
    ramdisk_size_arg,
    ramdisk_path_arg,
    backup_path_arg,
    namespace_arg,
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

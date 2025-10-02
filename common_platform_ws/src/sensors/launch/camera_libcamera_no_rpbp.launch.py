#!/usr/bin/env python3
"""
Libcamera launch file specifically designed to avoid RPBP pixel format warning.

This launch file uses multiple parameter combinations to force RGB888 format
and avoid the "Unsupported V4L2 pixel format RPBP" warning.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  # Declare launch arguments
  camera_id_arg = DeclareLaunchArgument(
    'camera_id',
    default_value='0',
    description='Camera ID (0 for first camera)'
  )
  
  width_arg = DeclareLaunchArgument(
    'width',
    default_value='640',
    description='Image width'
  )
  
  height_arg = DeclareLaunchArgument(
    'height',
    default_value='480',
    description='Image height'
  )
  
  use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation time'
  )
  
  # Get launch configurations
  camera_id = LaunchConfiguration('camera_id')
  width = LaunchConfiguration('width')
  height = LaunchConfiguration('height')
  use_sim_time = LaunchConfiguration('use_sim_time')
  
  # Handle namespace from environment variable
  ns = os.environ.get('ROS_NAMESPACE', '').strip()
  
  # Create camera node with multiple format parameters to avoid RPBP
  camera_node = Node(
    package='camera_ros',
    executable='camera_node',
    name='camera',
    namespace=ns if ns else '',  # Use ROS_NAMESPACE if set, otherwise empty
    parameters=[{
      'camera': camera_id,
      'width': width,
      'height': height,
      'use_sim_time': use_sim_time,
      # Multiple format parameters to force RGB888 and avoid RPBP
      'format': 'RGB888',
      'pixel_format': 'RGB888',
      'encoding': 'rgb8',
      'color_space': 'sRGB',
      'image_encoding': 'rgb8',
      'output_encoding': 'rgb8',
      # Additional parameters for stability
      'timeout': 5000,
      'retry_count': 3,
      'buffer_size': 2,
      # Force specific format settings
      'force_format': True,
      'preferred_format': 'RGB888',
    }],
    output='screen',
  )
  
  # Always return the same launch description - namespace handling is done by ROS2
  return LaunchDescription([
    camera_id_arg,
    width_arg,
    height_arg,
    use_sim_time_arg,
    camera_node
  ])

#!/usr/bin/env python3
"""
Fixed libcamera launch file that avoids pixel format issues.

This launch file uses specific parameters to avoid the "Unsupported V4L2 pixel format RPBP" warning.
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
  
  fps_arg = DeclareLaunchArgument(
    'fps',
    default_value='30',
    description='Frame rate'
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
  fps = LaunchConfiguration('fps')
  use_sim_time = LaunchConfiguration('use_sim_time')
  
  # Handle namespace from environment variable
  ns = os.environ.get('ROS_NAMESPACE', '').strip()
  
  # Create camera node with fixed pixel format
  camera_node = Node(
    package='camera_ros',
    executable='camera_node',
    name='camera',
    namespace='camera',  # This will be relative to the ROS_NAMESPACE
    parameters=[{
      'camera': camera_id,
      'width': width,
      'height': height,
      'fps': fps,
      'use_sim_time': use_sim_time,
      # Frame ID with namespace prefix for proper TF tree
      'frame_id': f'{ns}/camera_link' if ns else 'camera_link',
      # Fixed pixel format parameters to avoid RPBP warning
      'pixel_format': 'RGB888',
      'format': 'RGB888',
      'color_space': 'sRGB',
      'encoding': 'rgb8',
      # Additional parameters for stability
      'timeout': 5000,
      'retry_count': 3,
      'buffer_size': 2,
    }],
    output='screen',
  )
  
  # Always return the same launch description - namespace handling is done by ROS2
  return LaunchDescription([
    camera_id_arg,
    width_arg,
    height_arg,
    fps_arg,
    use_sim_time_arg,
    camera_node
  ])

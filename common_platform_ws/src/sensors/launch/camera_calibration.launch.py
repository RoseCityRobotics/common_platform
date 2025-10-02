#!/usr/bin/env python3
"""
Camera calibration launch file for creating camera_info and calibration data.
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
    default_value='1280',
    description='Image width'
  )
  
  height_arg = DeclareLaunchArgument(
    'height',
    default_value='720',
    description='Image height'
  )
  
  calibration_file_arg = DeclareLaunchArgument(
    'calibration_file',
    default_value='',
    description='Path to camera calibration file (YAML)'
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
  calibration_file = LaunchConfiguration('calibration_file')
  use_sim_time = LaunchConfiguration('use_sim_time')
  
  # Handle namespace from environment variable
  ns = os.environ.get('ROS_NAMESPACE', '').strip()
  
  # Create camera node
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
      # Frame ID with namespace prefix for proper TF tree
      'frame_id': f'{ns}/camera' if ns else 'camera',
      # Calibration file path
      'camera_info_url': calibration_file,
      # High-quality format settings
      'format': 'RGB888',
      'encoding': 'rgb8',
      'color_space': 'sRGB',
    }],
    output='screen',
  )
  
  # Camera info manager node for calibration
  camera_info_manager = Node(
    package='camera_info_manager',
    executable='camera_info_manager',
    name='camera_info_manager',
    namespace=ns if ns else '',
    parameters=[{
      'camera_info_url': calibration_file,
      'use_sim_time': use_sim_time,
    }],
    output='screen',
  )
  
  # Always return the same launch description - namespace handling is done by ROS2
  return LaunchDescription([
    camera_id_arg,
    width_arg,
    height_arg,
    calibration_file_arg,
    use_sim_time_arg,
    camera_node,
    camera_info_manager
  ])

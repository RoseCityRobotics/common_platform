#!/usr/bin/env python3
"""
High-quality camera launch file with optimized settings for better image quality.
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
    description='Image width (higher = better quality)'
  )
  
  height_arg = DeclareLaunchArgument(
    'height',
    default_value='720',
    description='Image height (higher = better quality)'
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
  
  # Create camera node with high-quality settings
  camera_node = Node(
    package='camera_ros',
    executable='camera_node',
    name='camera',
    namespace=ns if ns else '',  # Use ROS_NAMESPACE if set, otherwise empty
    parameters=[{
      'camera': camera_id,
      'width': width,
      'height': height,
      'fps': fps,
      'use_sim_time': use_sim_time,
      # Frame ID with namespace prefix for proper TF tree
      'frame_id': f'{ns}/camera' if ns else 'camera',
      # High-quality format settings
      'format': 'RGB888',
      'encoding': 'rgb8',
      'color_space': 'sRGB',
      # Camera quality settings
      'brightness': 0.5,      # Adjust brightness (0.0 to 1.0)
      'contrast': 1.0,        # Adjust contrast (0.0 to 2.0)
      'saturation': 1.0,      # Adjust saturation (0.0 to 2.0)
      'sharpness': 1.0,       # Adjust sharpness (0.0 to 2.0)
      'exposure_mode': 'auto', # Auto exposure
      'white_balance_mode': 'auto', # Auto white balance
      # Additional quality parameters
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

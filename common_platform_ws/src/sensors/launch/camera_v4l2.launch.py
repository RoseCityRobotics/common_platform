#!/usr/bin/env python3
"""
Launch file for V4L2 camera (alternative to libcamera).

This launch file uses v4l2_camera package which might be more stable
for the Pi HQ camera in some configurations.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  # Declare launch arguments
  device_arg = DeclareLaunchArgument(
    'device',
    default_value='/dev/video0',
    description='Camera device path'
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
  device = LaunchConfiguration('device')
  width = LaunchConfiguration('width')
  height = LaunchConfiguration('height')
  fps = LaunchConfiguration('fps')
  use_sim_time = LaunchConfiguration('use_sim_time')
  
  # Handle namespace from environment variable
  ns = os.environ.get('ROS_NAMESPACE', '').strip()
  
  # Create V4L2 camera node
  camera_node = Node(
    package='v4l2_camera',
    executable='v4l2_camera_node',
    name='camera',
    namespace='camera',  # This will be relative to the ROS_NAMESPACE
    parameters=[{
      'video_device': device,
      'image_size': [width, height],
      'fps': fps,
      'use_sim_time': use_sim_time,
      # Frame ID with namespace prefix for proper TF tree
      'frame_id': f'{ns}/camera_link' if ns else 'camera_link',
    }],
    output='screen',
  )
  
  # Always return the same launch description - namespace handling is done by ROS2
  return LaunchDescription([
    device_arg,
    width_arg,
    height_arg,
    fps_arg,
    use_sim_time_arg,
    camera_node
  ])

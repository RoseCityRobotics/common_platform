#!/usr/bin/env python3
"""
Standalone image rectification launch file.
Subscribes to existing camera topics and publishes rectified images.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  # Declare launch arguments
  input_image_topic_arg = DeclareLaunchArgument(
    'input_image_topic',
    default_value='camera/image_raw',
    description='Input image topic to rectify'
  )
  
  input_camera_info_topic_arg = DeclareLaunchArgument(
    'input_camera_info_topic',
    default_value='camera/camera_info',
    description='Input camera info topic'
  )
  
  output_image_topic_arg = DeclareLaunchArgument(
    'output_image_topic',
    default_value='camera/image_rect/image_raw',
    description='Output rectified image topic'
  )
  
  use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation time'
  )
  
  # Get launch configurations
  input_image_topic = LaunchConfiguration('input_image_topic')
  input_camera_info_topic = LaunchConfiguration('input_camera_info_topic')
  output_image_topic = LaunchConfiguration('output_image_topic')
  use_sim_time = LaunchConfiguration('use_sim_time')
  
  # Handle namespace from environment variable
  ns = os.environ.get('ROS_NAMESPACE', '').strip()
  
  # Image rectification node
  image_rect_node = Node(
    package='image_proc',
    executable='rectify_node',
    name='image_rectify',
    namespace=ns if ns else '',
    parameters=[{
      'use_sim_time': use_sim_time,
    }],
    remappings=[
      ('image', input_image_topic),
      ('camera_info', input_camera_info_topic),
      ('image_rect', output_image_topic),
    ],
    output='screen',
  )
  
  return LaunchDescription([
    input_image_topic_arg,
    input_camera_info_topic_arg,
    output_image_topic_arg,
    use_sim_time_arg,
    image_rect_node
  ])

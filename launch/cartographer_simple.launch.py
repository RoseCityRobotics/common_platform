"""
Simple launch file for cartographer_node with dynamic namespace from ROS_NAME.
This is a simplified version that uses the ROS_NAME environment variable for topic remapping.
"""

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
import os

def generate_launch_description():
  # Get the namespace from ROS_NAME environment variable, default to ''
  namespace = os.getenv('ROS_NAME', '')
  
  # Cartographer node with proper remapping using ROS_NAME
  cartographer_node = Node(
    package='cartographer_ros',
    executable='cartographer_node',
    name='cartographer_node',
    parameters=[{
      'use_sim_time': False
    }],
    arguments=[
      '-configuration_directory', '/home/rcr/ros2_ws',
      '-configuration_basename', 'common_platform.lua'
    ],
    output='screen'
  )

  return LaunchDescription([
    PushRosNamespace(namespace),
    cartographer_node
  ])

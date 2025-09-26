#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time')
  configuration_directory = LaunchConfiguration('configuration_directory')
  configuration_basename = LaunchConfiguration('configuration_basename')
  resolution = LaunchConfiguration('resolution')
  publish_period_sec = LaunchConfiguration('publish_period_sec')
  scan_topic = LaunchConfiguration('scan_topic')

  declare_use_sim_time = DeclareLaunchArgument(
    'use_sim_time', default_value='false', description='Use simulation clock')

  default_config_dir = os.path.join(
    get_package_share_directory('common_platform'), 'config', 'cartographer')
  declare_config_dir = DeclareLaunchArgument(
    'configuration_directory', default_value=default_config_dir,
    description='Directory containing Cartographer Lua configs')

  declare_config_base = DeclareLaunchArgument(
    'configuration_basename', default_value='cartographer_2d.lua',
    description='Cartographer configuration basename')

  declare_resolution = DeclareLaunchArgument(
    'resolution', default_value='0.05', description='Occupancy grid resolution (m)')
  declare_publish_period = DeclareLaunchArgument(
    'publish_period_sec', default_value='1.0', description='Occupancy grid publish period (s)')
  declare_scan_topic = DeclareLaunchArgument(
    'scan_topic', default_value='scan', description='LaserScan topic to subscribe to')

  cartographer_node = Node(
    package='cartographer_ros',
    executable='cartographer_node',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
    arguments=['-configuration_directory', configuration_directory,
               '-configuration_basename', configuration_basename]
  )

  occupancy_grid_node = Node(
    package='cartographer_ros',
    executable='cartographer_occupancy_grid_node',
    output='screen',
    parameters=[{
      'use_sim_time': use_sim_time,
      'resolution': resolution,
      'publish_period_sec': publish_period_sec
    }]
  )

  actions = [
    declare_use_sim_time,
    declare_config_dir,
    declare_config_base,
    declare_resolution,
    declare_publish_period,
    declare_scan_topic,
    cartographer_node,
    occupancy_grid_node,
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



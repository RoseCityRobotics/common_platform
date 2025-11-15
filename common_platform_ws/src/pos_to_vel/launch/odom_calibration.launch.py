import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():
  # Declare launch arguments
  forward_distance_arg = DeclareLaunchArgument(
    'forward_distance',
    default_value='0.05',
    description='Forward distance for each action in meters (default: 0.05 = 5cm)'
  )
  
  turn_angle_arg = DeclareLaunchArgument(
    'turn_angle',
    default_value='1.5708',
    description='Turn angle in radians (default: 1.5708 = 90 degrees)'
  )
  
  linear_speed_arg = DeclareLaunchArgument(
    'linear_speed',
    default_value='0.1',
    description='Linear speed for forward movement in m/s'
  )
  
  angular_speed_arg = DeclareLaunchArgument(
    'angular_speed',
    default_value='0.5',
    description='Angular speed for turning in rad/s'
  )
  
  scan_topic_arg = DeclareLaunchArgument(
    'scan_topic',
    default_value='scan',
    description='Lidar scan topic name'
  )
  
  odom_topic_arg = DeclareLaunchArgument(
    'odom_topic',
    default_value='odom',
    description='Odometry topic name'
  )
  
  cmd_vel_topic_arg = DeclareLaunchArgument(
    'cmd_vel_topic',
    default_value='cmd_vel',
    description='Command velocity topic name'
  )
  
  namespace_arg = DeclareLaunchArgument(
    'namespace',
    default_value='',
    description='ROS namespace (can also use ROS_NAME environment variable)'
  )
  
  device_path_arg = DeclareLaunchArgument(
    'device_path',
    default_value='/dev/input/event0',
    description='Path to evdev input device (e.g., /dev/input/event0 for USB keyboard dongle)'
  )
  
  grab_device_arg = DeclareLaunchArgument(
    'grab_device',
    default_value='true',
    description='Grab device exclusively to prevent events from going to console/X'
  )
  
  debug_arg = DeclareLaunchArgument(
    'debug',
    default_value='true',
    description='Enable debug logging for keyboard events'
  )
  
  # Get launch configurations
  forward_distance = LaunchConfiguration('forward_distance')
  turn_angle = LaunchConfiguration('turn_angle')
  linear_speed = LaunchConfiguration('linear_speed')
  angular_speed = LaunchConfiguration('angular_speed')
  scan_topic = LaunchConfiguration('scan_topic')
  odom_topic = LaunchConfiguration('odom_topic')
  cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
  namespace = LaunchConfiguration('namespace')
  device_path = LaunchConfiguration('device_path')
  grab_device = LaunchConfiguration('grab_device')
  debug = LaunchConfiguration('debug')
  
  # Create the calibration node
  calibration_node = Node(
    package='pos_to_vel',
    executable='odom_calibration_node',
    name='odom_calibration_node',
    output='screen',
    parameters=[{
      'forward_distance': forward_distance,
      'turn_angle': turn_angle,
      'linear_speed': linear_speed,
      'angular_speed': angular_speed,
      'scan_topic': scan_topic,
      'odom_topic': odom_topic,
      'cmd_vel_topic': cmd_vel_topic,
      'namespace': namespace,
      'device_path': device_path,
      'grab_device': grab_device,
      'debug': debug,
    }]
  )
  
  # Handle namespace from environment variable or launch argument
  ns = os.environ.get('ROS_NAME', '').strip()
  if ns:
    return LaunchDescription([
      forward_distance_arg,
      turn_angle_arg,
      linear_speed_arg,
      angular_speed_arg,
      scan_topic_arg,
      odom_topic_arg,
      cmd_vel_topic_arg,
      namespace_arg,
      device_path_arg,
      grab_device_arg,
      debug_arg,
      GroupAction([
        PushRosNamespace(ns),
        calibration_node
      ])
    ])
  else:
    return LaunchDescription([
      forward_distance_arg,
      turn_angle_arg,
      linear_speed_arg,
      angular_speed_arg,
      scan_topic_arg,
      odom_topic_arg,
      cmd_vel_topic_arg,
      namespace_arg,
      device_path_arg,
      grab_device_arg,
      debug_arg,
      calibration_node
    ])


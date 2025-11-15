import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():
  # Declare launch arguments
  model_path_arg = DeclareLaunchArgument(
    'model_path',
    default_value='q-tabular.pkl',
    description='Path to the trained Q-table model pickle file'
  )
  
  forward_distance_arg = DeclareLaunchArgument(
    'forward_distance',
    default_value='0.15',
    description='Forward distance for each action in meters (default: 0.15 = 15cm)'
  )
  
  turn_angle_arg = DeclareLaunchArgument(
    'turn_angle',
    default_value='1.5708',
    description='Turn angle in radians (default: 1.5708 = 90 degrees)'
  )
  
  linear_speed_arg = DeclareLaunchArgument(
    'linear_speed',
    default_value='0.3',
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
  
  action_rate_arg = DeclareLaunchArgument(
    'action_rate',
    default_value='1.0',
    description='Action selection rate in Hz (actions per second)'
  )
  
  cell_size_arg = DeclareLaunchArgument(
    'cell_size',
    default_value='0.10',
    description='State discretization cell size in meters'
  )
  
  collision_threshold_arg = DeclareLaunchArgument(
    'collision_threshold',
    default_value='0.05',
    description='Minimum forward clearance in meters before collision is detected (default: 0.05m)'
  )
  
  forward_scan_angle_range_arg = DeclareLaunchArgument(
    'forward_scan_angle_range',
    default_value='30.0',
    description='Degrees to check on each side of forward direction for collision detection (default: 30°)'
  )
  
  # Get launch configurations
  model_path = LaunchConfiguration('model_path')
  forward_distance = LaunchConfiguration('forward_distance')
  turn_angle = LaunchConfiguration('turn_angle')
  linear_speed = LaunchConfiguration('linear_speed')
  angular_speed = LaunchConfiguration('angular_speed')
  scan_topic = LaunchConfiguration('scan_topic')
  odom_topic = LaunchConfiguration('odom_topic')
  cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
  namespace = LaunchConfiguration('namespace')
  action_rate = LaunchConfiguration('action_rate')
  cell_size = LaunchConfiguration('cell_size')
  collision_threshold = LaunchConfiguration('collision_threshold')
  forward_scan_angle_range = LaunchConfiguration('forward_scan_angle_range')
  
  # Create the reinforcement learning node
  rl_node = Node(
    package='reinforcement_learning',
    executable='reinforcement_learning_node',
    name='reinforcement_learning_node',
    output='screen',
    parameters=[{
      'model_path': model_path,
      'forward_distance': forward_distance,
      'turn_angle': turn_angle,
      'linear_speed': linear_speed,
      'angular_speed': angular_speed,
      'scan_topic': scan_topic,
      'odom_topic': odom_topic,
      'cmd_vel_topic': cmd_vel_topic,
      'namespace': namespace,
      'action_rate': action_rate,
      'cell_size': cell_size,
      'collision_threshold': collision_threshold,
      'forward_scan_angle_range': forward_scan_angle_range,
    }]
  )
  
  # Handle namespace from environment variable or launch argument
  ns = os.environ.get('ROS_NAME', '').strip()
  if ns:
    return LaunchDescription([
      model_path_arg,
      forward_distance_arg,
      turn_angle_arg,
      linear_speed_arg,
      angular_speed_arg,
      scan_topic_arg,
      odom_topic_arg,
      cmd_vel_topic_arg,
      namespace_arg,
      action_rate_arg,
      cell_size_arg,
      collision_threshold_arg,
      forward_scan_angle_range_arg,
      GroupAction([
        PushRosNamespace(ns),
        rl_node
      ])
    ])
  else:
    return LaunchDescription([
      model_path_arg,
      forward_distance_arg,
      turn_angle_arg,
      linear_speed_arg,
      angular_speed_arg,
      scan_topic_arg,
      odom_topic_arg,
      cmd_vel_topic_arg,
      namespace_arg,
      action_rate_arg,
      cell_size_arg,
      collision_threshold_arg,
      forward_scan_angle_range_arg,
      rl_node
    ])


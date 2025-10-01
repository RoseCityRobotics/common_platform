import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
  # Get the launch directory
  data_recorder_dir = get_package_share_directory('data_recorder')
  
  # Declare launch arguments
  output_dir_arg = DeclareLaunchArgument(
    'output_dir',
    default_value='/home/rcr/teleop_data',
    description='Directory to save recorded data'
  )
  
  camera_topic_arg = DeclareLaunchArgument(
    'camera_topic',
    default_value='/camera/image_raw',
    description='Camera topic to record'
  )
  
  cmd_vel_topic_arg = DeclareLaunchArgument(
    'cmd_vel_topic',
    default_value='/cmd_vel',
    description='Command velocity topic to record'
  )
  
  record_rate_arg = DeclareLaunchArgument(
    'record_rate',
    default_value='30.0',
    description='Recording rate in Hz'
  )
  
  auto_start_arg = DeclareLaunchArgument(
    'auto_start',
    default_value='false',
    description='Auto-start recording when node launches'
  )
  
  use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation time'
  )
  
  # Get launch configurations
  output_dir = LaunchConfiguration('output_dir')
  camera_topic = LaunchConfiguration('camera_topic')
  cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
  record_rate = LaunchConfiguration('record_rate')
  auto_start = LaunchConfiguration('auto_start')
  use_sim_time = LaunchConfiguration('use_sim_time')
  
  # Handle namespace for topics
  ns = os.environ.get('ROS_NAMESPACE', '').strip()
  if ns:
    # Prepend namespace to topic names
    camera_topic_with_ns = f'/{ns}{camera_topic}' if not str(camera_topic).startswith('/') else f'/{ns}{camera_topic}'
    cmd_vel_topic_with_ns = f'/{ns}{cmd_vel_topic}' if not str(cmd_vel_topic).startswith('/') else f'/{ns}{cmd_vel_topic}'
  else:
    camera_topic_with_ns = camera_topic
    cmd_vel_topic_with_ns = cmd_vel_topic
  
  # Data recorder node
  data_recorder_node = Node(
    package='data_recorder',
    executable='data_recorder_node',
    name='data_recorder',
    output='screen',
    parameters=[{
      'output_dir': output_dir,
      'camera_topic': camera_topic_with_ns,
      'cmd_vel_topic': cmd_vel_topic_with_ns,
      'record_rate': record_rate,
      'auto_start': auto_start,
      'use_sim_time': use_sim_time,
    }],
    remappings=[
      # Add any topic remappings here if needed
    ]
  )
  
  # Handle namespace for node placement
  if ns:
    return LaunchDescription([
      output_dir_arg,
      camera_topic_arg,
      cmd_vel_topic_arg,
      record_rate_arg,
      auto_start_arg,
      use_sim_time_arg,
      GroupAction([
        PushRosNamespace(ns),
        data_recorder_node
      ])
    ])
  else:
    return LaunchDescription([
      output_dir_arg,
      camera_topic_arg,
      cmd_vel_topic_arg,
      record_rate_arg,
      auto_start_arg,
      use_sim_time_arg,
      data_recorder_node
    ])

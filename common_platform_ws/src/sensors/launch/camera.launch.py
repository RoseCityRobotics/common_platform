import os
from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_nodes(context):
  # Get launch configurations (evaluate them to get actual values)
  camera_id_str = context.launch_configurations.get('camera_id', '0')
  width_str = context.launch_configurations.get('width', '640')
  height_str = context.launch_configurations.get('height', '480')
  format = context.launch_configurations.get('format', 'RGB888')
  use_sim_time_str = context.launch_configurations.get('use_sim_time', 'false')
  flip_enabled_str = context.launch_configurations.get('flip_enabled', 'false')
  flip_code_str = context.launch_configurations.get('flip_code', '-1')
  
  # Convert parameters to proper types
  try:
    camera_id = int(camera_id_str)
  except (ValueError, TypeError):
    camera_id = 0
  
  try:
    width = int(width_str)
  except (ValueError, TypeError):
    width = 640
  
  try:
    height = int(height_str)
  except (ValueError, TypeError):
    height = 480
  
  # Convert flip_enabled string to boolean
  flip_enabled_bool = flip_enabled_str.lower() in ('true', '1', 'yes', 'on')
  
  # Convert flip_code to int
  try:
    flip_code_int = int(flip_code_str)
  except (ValueError, TypeError):
    flip_code_int = -1
  
  # Convert use_sim_time to boolean
  use_sim_time_bool = use_sim_time_str.lower() in ('true', '1', 'yes', 'on')
  
  # Handle namespace from environment variable
  ns = os.environ.get('ROS_NAMESPACE', '').strip()
  
  # Create regular camera node for libcamera
  # Camera publishes to intermediate topic, flip node will handle the final output
  camera_node = Node(
    package='camera_ros',
    executable='camera_node',
    name='camera',
    parameters=[{
      'camera': camera_id,
      'width': width,
      'height': height,
      'format': 'RGB888',  # Force RGB888 format to avoid RPBP warning
      'use_sim_time': use_sim_time_bool,
      # Frame ID with namespace prefix for proper TF tree
      'frame_id': f'{ns}/camera' if ns else 'camera',
      # Add parameters to help with device busy issues
      'timeout': 5000,  # 5 second timeout
      'retry_count': 3,  # Retry 3 times
      # Additional format parameters for camera_ros
      'encoding': 'rgb8',
      'color_space': 'sRGB',
    }],
    remappings=[('camera/image_raw', 'camera/image_raw_unflipped')],
    output='screen',
  )
  
  # Create image flip node
  # When flip_enabled is false, it will pass through images without flipping
  flip_node = Node(
    package='sensors',
    executable='image_flip_node',
    name='image_flip',
    parameters=[{
      'flip_enabled': flip_enabled_bool,
      'flip_code': flip_code_int,
      'input_topic': 'camera/image_raw_unflipped',
      'output_topic': 'camera/image_raw',
      'use_sim_time': use_sim_time_bool,
    }],
    output='screen',
  )
  
  nodes_to_launch = [camera_node, flip_node]
  
  # Handle namespace properly using GroupAction and PushRosNamespace
  if ns:
    return [
      GroupAction([
        PushRosNamespace(ns),
        *nodes_to_launch
      ])
    ]
  else:
    return nodes_to_launch


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
  
  format_arg = DeclareLaunchArgument(
    'format',
    default_value='RGB888',
    description='Pixel format (RGB888, YUV420, etc.)'
  )
  
  use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation time'
  )
  
  flip_enabled_arg = DeclareLaunchArgument(
    'flip_enabled',
    default_value='false',
    description='Enable image flipping (for upside-down camera mounting)'
  )
  
  flip_code_arg = DeclareLaunchArgument(
    'flip_code',
    default_value='-1',
    description='Flip code: 0=vertical, 1=horizontal, -1=both axes (180° rotation)'
  )
  
  return LaunchDescription([
    camera_id_arg,
    width_arg,
    height_arg,
    format_arg,
    use_sim_time_arg,
    flip_enabled_arg,
    flip_code_arg,
    OpaqueFunction(function=generate_launch_nodes)
  ])

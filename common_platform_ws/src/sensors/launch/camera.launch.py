import os
from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

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
  
  # Get launch configurations
  camera_id = LaunchConfiguration('camera_id')
  width = LaunchConfiguration('width')
  height = LaunchConfiguration('height')
  format = LaunchConfiguration('format')
  use_sim_time = LaunchConfiguration('use_sim_time')
  flip_enabled = LaunchConfiguration('flip_enabled')
  flip_code = LaunchConfiguration('flip_code')
  
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
      'use_sim_time': use_sim_time,
      # Frame ID with namespace prefix for proper TF tree
      'frame_id': f'{ns}/camera' if ns else 'camera',
      # Add parameters to help with device busy issues
      'timeout': 5000,  # 5 second timeout
      'retry_count': 3,  # Retry 3 times
      # Additional format parameters for camera_ros
      'encoding': 'rgb8',
      'color_space': 'sRGB',
    }],
    remappings=[('image_raw', 'camera/image_raw_unflipped')],
    output='screen',
  )
  
  # Create image flip node
  # When flip_enabled is false, it will pass through images without flipping
  flip_node = Node(
    package='sensors',
    executable='image_flip_node',
    name='image_flip',
    parameters=[{
      'flip_enabled': flip_enabled,
      'flip_code': flip_code,
      'input_topic': 'camera/image_raw_unflipped',
      'output_topic': 'camera/image_raw',
      'use_sim_time': use_sim_time,
    }],
    output='screen',
  )
  
  nodes_to_launch = [camera_node, flip_node]
  
  # Handle namespace properly using GroupAction and PushRosNamespace
  launch_args = [
    camera_id_arg,
    width_arg,
    height_arg,
    format_arg,
    use_sim_time_arg,
    flip_enabled_arg,
    flip_code_arg,
  ]
  
  if ns:
    return LaunchDescription(launch_args + [
      GroupAction([
        PushRosNamespace(ns),
        *nodes_to_launch
      ])
    ])
  else:
    return LaunchDescription(launch_args + nodes_to_launch)

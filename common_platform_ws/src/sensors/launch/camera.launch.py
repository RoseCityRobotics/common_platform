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
  
  rate_arg = DeclareLaunchArgument(
    'rate',
    default_value='0',
    description='Publishing rate in Hz (0 = camera native rate)'
  )
  
  # Get launch configurations
  camera_id = LaunchConfiguration('camera_id')
  width = LaunchConfiguration('width')
  height = LaunchConfiguration('height')
  format = LaunchConfiguration('format')
  use_sim_time = LaunchConfiguration('use_sim_time')
  rate = LaunchConfiguration('rate')
  
  # Handle namespace from environment variable
  ns = os.environ.get('ROS_NAMESPACE', '').strip()
  
  # Create regular camera node for libcamera
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
      # Rate parameter (if supported by camera_ros)
      'rate': rate,
    }],
    output='screen',
  )
  
  # Handle namespace properly using GroupAction and PushRosNamespace
  if ns:
    return LaunchDescription([
      camera_id_arg,
      width_arg,
      height_arg,
      format_arg,
      use_sim_time_arg,
      rate_arg,
      GroupAction([
        PushRosNamespace(ns),
        camera_node
      ])
    ])
  else:
    return LaunchDescription([
      camera_id_arg,
      width_arg,
      height_arg,
      format_arg,
      use_sim_time_arg,
      rate_arg,
      camera_node
    ])

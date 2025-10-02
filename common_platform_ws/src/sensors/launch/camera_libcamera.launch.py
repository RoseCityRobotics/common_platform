import os
from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

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
  
  # Get launch configurations
  camera_id = LaunchConfiguration('camera_id')
  width = LaunchConfiguration('width')
  height = LaunchConfiguration('height')
  format = LaunchConfiguration('format')
  use_sim_time = LaunchConfiguration('use_sim_time')
  
  # Handle namespace from environment variable
  ns = os.environ.get('ROS_NAMESPACE', '').strip()
  
  # Create composable camera node for libcamera
  # Use namespace from ROS_NAMESPACE environment variable
  camera_node = ComposableNode(
    package='camera_ros',
    plugin='camera::CameraNode',
    name='camera',
    namespace=ns if ns else '',  # Use ROS_NAMESPACE if set, otherwise empty
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
    extra_arguments=[{'use_intra_process_comms': True}],
  )
  
  # Create container for composable nodes
  # Use namespace from ROS_NAMESPACE environment variable
  camera_container = ComposableNodeContainer(
    name='camera_container',
    namespace=ns if ns else '',  # Use ROS_NAMESPACE if set, otherwise empty
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[camera_node],
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
  )
  
  # Always return the same launch description - namespace handling is done by ROS2
  return LaunchDescription([
    camera_id_arg,
    width_arg,
    height_arg,
    format_arg,
    use_sim_time_arg,
    camera_container
  ])

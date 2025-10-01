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
    default_value='',
    description='Pixel format (empty for default)'
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
  
  # Create composable camera node for libcamera
  camera_node = ComposableNode(
    package='camera_ros',
    plugin='camera::CameraNode',
    name='camera',
    namespace='camera',
    parameters=[{
      'camera': camera_id,
      'width': width,
      'height': height,
      'format': format,
      'use_sim_time': use_sim_time,
    }],
    extra_arguments=[{'use_intra_process_comms': True}],
  )
  
  # Create container for composable nodes
  camera_container = ComposableNodeContainer(
    name='camera_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[camera_node],
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
  )
  
  # Handle namespace
  ns = os.environ.get('ROS_NAMESPACE', '').strip()
  if ns:
    return LaunchDescription([
      camera_id_arg,
      width_arg,
      height_arg,
      format_arg,
      use_sim_time_arg,
      GroupAction([
        PushRosNamespace(ns),
        camera_container
      ])
    ])
  else:
    return LaunchDescription([
      camera_id_arg,
      width_arg,
      height_arg,
      format_arg,
      use_sim_time_arg,
      camera_container
    ])

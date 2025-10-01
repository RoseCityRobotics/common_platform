import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
  # Get package directories
  data_recorder_dir = get_package_share_directory('data_recorder')
  common_platform_dir = get_package_share_directory('common_platform')
  
  # Declare launch arguments
  camera_type_arg = DeclareLaunchArgument(
    'camera_type',
    default_value='libcamera',
    description='Camera type: v4l2 or libcamera'
  )
  
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
  
  # Camera-specific arguments
  camera_id_arg = DeclareLaunchArgument(
    'camera_id',
    default_value='0',
    description='Camera ID for libcamera'
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
  
  # Get launch configurations
  camera_type = LaunchConfiguration('camera_type')
  output_dir = LaunchConfiguration('output_dir')
  camera_topic = LaunchConfiguration('camera_topic')
  cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
  record_rate = LaunchConfiguration('record_rate')
  auto_start = LaunchConfiguration('auto_start')
  use_sim_time = LaunchConfiguration('use_sim_time')
  camera_id = LaunchConfiguration('camera_id')
  width = LaunchConfiguration('width')
  height = LaunchConfiguration('height')
  
  # Handle namespace for topics
  ns = os.environ.get('ROS_NAMESPACE', '').strip()
  if ns:
    # Prepend namespace to topic names
    camera_topic_with_ns = f'/{ns}{camera_topic}' if not str(camera_topic).startswith('/') else f'/{ns}{camera_topic}'
    cmd_vel_topic_with_ns = f'/{ns}{cmd_vel_topic}' if not str(cmd_vel_topic).startswith('/') else f'/{ns}{cmd_vel_topic}'
  else:
    camera_topic_with_ns = camera_topic
    cmd_vel_topic_with_ns = cmd_vel_topic
  
  # Choose camera launch based on type
  if camera_type == 'libcamera':
    camera_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        os.path.join(common_platform_dir, 'launch', 'camera_libcamera.launch.py')
      ]),
      launch_arguments={
        'camera_id': camera_id,
        'width': width,
        'height': height,
        'use_sim_time': use_sim_time,
      }.items()
    )
  else:  # v4l2
    camera_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        os.path.join(common_platform_dir, 'launch', 'camera.launch.py')
      ]),
      launch_arguments={
        'use_sim_time': use_sim_time,
      }.items()
    )
  
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
      'camera_type': camera_type,
      'camera_id': camera_id,
    }],
  )
  
  # Handle namespace for node placement
  if ns:
    return LaunchDescription([
      camera_type_arg,
      output_dir_arg,
      camera_topic_arg,
      cmd_vel_topic_arg,
      record_rate_arg,
      auto_start_arg,
      use_sim_time_arg,
      camera_id_arg,
      width_arg,
      height_arg,
      GroupAction([
        PushRosNamespace(ns),
        camera_launch,
        data_recorder_node
      ])
    ])
  else:
    return LaunchDescription([
      camera_type_arg,
      output_dir_arg,
      camera_topic_arg,
      cmd_vel_topic_arg,
      record_rate_arg,
      auto_start_arg,
      use_sim_time_arg,
      camera_id_arg,
      width_arg,
      height_arg,
      camera_launch,
      data_recorder_node
    ])

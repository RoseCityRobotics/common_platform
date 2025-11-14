#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get namespace from ROS_NAMESPACE environment variable
    namespace = os.environ.get('ROS_NAMESPACE', '').strip()
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/rcr/repos/common_platform/models/imitation_learning.onnx',
            description='Path to the ONNX model file'
        ),
        DeclareLaunchArgument(
            'input_width',
            default_value='224',
            description='Input image width'
        ),
        DeclareLaunchArgument(
            'input_height',
            default_value='224',
            description='Input image height'
        ),
        DeclareLaunchArgument(
            'sequence_length',
            default_value='10',
            description='Sequence length for temporal model'
        ),
        DeclareLaunchArgument(
            'max_linear_velocity',
            default_value='0.5',
            description='Maximum linear velocity (m/s)'
        ),
        DeclareLaunchArgument(
            'max_angular_velocity',
            default_value='1.5',
            description='Maximum angular velocity (rad/s)'
        ),
        DeclareLaunchArgument(
            'linear_velocity_threshold',
            default_value='0.01',
            description='Threshold below which linear velocity is set to 0 (m/s)'
        ),
        DeclareLaunchArgument(
            'angular_velocity_threshold',
            default_value='0.05',
            description='Threshold below which angular velocity is set to 0 (rad/s)'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='30.0',
            description='Publishing rate for cmd_vel (Hz)'
        ),
        DeclareLaunchArgument(
            'stats_report_interval',
            default_value='10.0',
            description='Interval for reporting inference statistics (seconds)'
        ),
        DeclareLaunchArgument(
            'max_inference_time_ms',
            default_value='33.0',
            description='Maximum expected inference time in milliseconds (for 30 Hz, ~33 ms)'
        ),
        Node(
            package='imitation_learning',
            executable='imitation_learning_node',
            namespace=namespace,
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'input_width': LaunchConfiguration('input_width'),
                'input_height': LaunchConfiguration('input_height'),
                'sequence_length': LaunchConfiguration('sequence_length'),
                'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
                'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
                'linear_velocity_threshold': LaunchConfiguration('linear_velocity_threshold'),
                'angular_velocity_threshold': LaunchConfiguration('angular_velocity_threshold'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'stats_report_interval': LaunchConfiguration('stats_report_interval'),
                'max_inference_time_ms': LaunchConfiguration('max_inference_time_ms'),
            }]
        )
    ])


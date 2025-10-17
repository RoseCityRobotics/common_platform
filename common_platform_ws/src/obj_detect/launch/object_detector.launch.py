#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get namespace from ROS_NAMESPACE environment variable
    namespace = os.environ.get('ROS_NAMESPACE', '')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/rcr/repos/common_platform/models/yolov11n_2cls.hef',
            description='Path to the HEF model file'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            description='Confidence threshold for detections'
        ),
        DeclareLaunchArgument(
            'input_width',
            default_value='640',
            description='Input image width'
        ),
        DeclareLaunchArgument(
            'input_height',
            default_value='640',
            description='Input image height'
        ),
        DeclareLaunchArgument(
            'hailort_log_path',
            default_value='/tmp/hailort.log',
            description='Path for HailoRT log file'
        ),
        Node(
            package='obj_detect',
            executable='object_detector',
            name='object_detector',
            namespace=namespace,
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'input_width': LaunchConfiguration('input_width'),
                'input_height': LaunchConfiguration('input_height'),
                'hailort_log_path': LaunchConfiguration('hailort_log_path'),
            }]
        )
    ])

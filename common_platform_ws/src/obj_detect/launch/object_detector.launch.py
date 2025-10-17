#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
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
            'nms_threshold',
            default_value='0.4',
            description='NMS threshold for post-processing'
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
        Node(
            package='obj_detect',
            executable='object_detector',
            name='object_detector',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'nms_threshold': LaunchConfiguration('nms_threshold'),
                'input_width': LaunchConfiguration('input_width'),
                'input_height': LaunchConfiguration('input_height'),
            }]
        )
    ])

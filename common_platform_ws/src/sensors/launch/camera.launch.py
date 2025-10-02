import os

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():



    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output='screen',
        namespace='camera',
        parameters=[{
            'image_size': [640,480],
            'time_per_frame': [1, 6],
            'camera_frame_id': 'camera_link_optical'
            }]
    )

    ns = os.environ.get('ROS_NAMESPACE', '').strip()
    if ns:
        return LaunchDescription([
            GroupAction([
                PushRosNamespace(ns),
                camera_node
            ])
        ])
    else:
        return LaunchDescription([
            camera_node
        ])

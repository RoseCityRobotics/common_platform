#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    package_name = 'common_platform'
    
    # Launch robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ])
    )
    
    # Joint State Publisher (for wheel joints)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    actions = [
        rsp,
        joint_state_publisher,
    ]

    ns = os.environ.get('ROS_NAMESPACE', '').strip()
    if ns:
        return LaunchDescription([
            GroupAction([
                PushRosNamespace(ns),
                *actions
            ])
        ])
    else:
        return LaunchDescription(actions)

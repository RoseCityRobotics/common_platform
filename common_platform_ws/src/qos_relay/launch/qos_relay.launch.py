import os
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():
    qos_relay_node = Node(
        package='qos_relay',
        executable='qos_relay',
        output='screen'
    )
    
    ns = os.environ.get('ROS_NAME', '').strip()
    if ns:
        return LaunchDescription([
            GroupAction([
                PushRosNamespace(ns),
                qos_relay_node
            ])
        ])
    else:
        return LaunchDescription([qos_relay_node])

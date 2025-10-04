import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    device_arg = DeclareLaunchArgument(
        'device_path',
        default_value='/dev/input/by-id/usb-Logitech_USB_Receiver-if01-event-kbd'
    )
    topic_arg = DeclareLaunchArgument('topic', default_value='cmd_vel')

    ns_arg = DeclareLaunchArgument('namespace', default_value='')

    # Prefer ROS_NAMESPACE environment variable if set
    ns_env = os.environ.get('ROS_NAMESPACE', '')
    node_namespace = ns_env if ns_env else LaunchConfiguration('namespace')
    print(f'node_namespace: {node_namespace}')

    return LaunchDescription([
        device_arg, topic_arg, ns_arg,
        Node(
            package='evdev_teleop',
            executable='evdev_teleop',
            name='evdev_teleop',
            namespace=node_namespace,
            parameters=[{
                'device_path': LaunchConfiguration('device_path'),
                'topic': LaunchConfiguration('topic'),
                'linear_speed': 0.3,
                'angular_speed': 1.2,
                'repeat_hz': 20.0,
                'grab_device': True,  # keeps SSH keyboard unaffected
            }],
            output='screen'
        )
    ])


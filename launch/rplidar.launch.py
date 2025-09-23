import os
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )

    ns = os.environ.get('ROS_NAMESPACE', '').strip()
    if ns:
        return LaunchDescription([
            GroupAction([
                PushRosNamespace(ns),
                lidar_node
            ])
        ])
    else:
        return LaunchDescription([lidar_node])

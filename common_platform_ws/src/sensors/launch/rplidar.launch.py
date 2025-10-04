import os
from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')

    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
        description='Serial device path for RPLidar')

    declare_serial_baud = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Serial baudrate for RPLidar')

    declare_frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='Frame id for the laser scans')

    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )

    ns = os.environ.get('ROS_NAMESPACE', '').strip()
    if ns:
        return LaunchDescription([
            declare_serial_port,
            declare_serial_baud,
            declare_frame_id,
            GroupAction([
                PushRosNamespace(ns),
                lidar_node
            ])
        ])
    else:
        return LaunchDescription([
            declare_serial_port,
            declare_serial_baud,
            declare_frame_id,
            lidar_node
        ])
